# grid runner shared by vicon_grid and test_grid (wheel odom)
# initialize(), then start_control_loop() plans grid, control_tick() runs one command at a time

import math
from collections import deque
import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_srvs.srv import SetBool
from nav_utils import (TimedNode, TwistSender, call_service, clamp, ignore_stop_signals, init_ros, normalize,
                       send_zero_burst, stop_on_signals)


PEAK_FACTOR = 35.0/16.0    # velocity_factor at peak (tau=0.5)
HEADING_GAIN = 1.8         # yaw P-gain for heading hold
LATERAL_GAIN = 2.0         # lateral offset to yaw error
LATERAL_CLAMP = 0.35       # max lateral error [m]
HEADING_FIX_LIMIT = 0.5    # heading_fix clamp


def world_to_body(vx_world, vy_world, yaw):
    # rotate world velocity into robot body frame (omni only)
    c, s = math.cos(yaw), math.sin(yaw)
    vx_body = vx_world * c + vy_world * s
    vy_body = -vx_world * s + vy_world * c
    return vx_body, vy_body


def position_factor(tau):
    tau = clamp(tau, 0.0, 1.0)
    return 35 * tau**4 - 84 * tau**5 + 70 * tau**6 - 20 * tau**7


def velocity_factor(tau):
    tau = clamp(tau, 0.0, 1.0)
    return 140 * tau**3 - 420 * tau**4 + 420 * tau**5 - 140 * tau**6


def duration_for_peak(distance, max_speed):
    distance = abs(distance)
    if distance <= 0 or max_speed <= 0:
        return 0.0
    return PEAK_FACTOR * distance / max_speed


def add_grid_arguments(p, *, position_tol, return_position_tol, angle_tol, homing_timeout):
    p.add_argument('--platform', choices=['diff', 'omni'], default='diff')
    p.add_argument('--grid-size', type=int, default=4)  # rows (height points) width set by --grid-cols
    p.add_argument('--grid-cols', type=int, default=None)  # columns (width points) default = --grid-size (square)
    p.add_argument('--cell-size', type=float, default=0.25)
    p.add_argument('--cell-size-x', type=float, default=None)
    p.add_argument('--cell-size-y', type=float, default=None)
    p.add_argument('--max-speed', type=float, default=0.1)
    p.add_argument('--turn-speed', type=float, default=0.25)
    p.add_argument('--corr-gain', type=float, default=1.5)
    p.add_argument('--corr-damping-gain', type=float, default=0.3)
    p.add_argument('--corr-filter-tau', type=float, default=0.08)
    p.add_argument('--position-tol', type=float, default=position_tol)
    p.add_argument('--return-position-tol', type=float, default=return_position_tol)
    p.add_argument('--angle-tol', type=float, default=angle_tol)
    p.add_argument('--rate', type=float, default=25.0)
    p.add_argument('--homing-timeout', type=float, default=homing_timeout)
    p.add_argument('--return-home-max-retries', type=int, default=8)
    p.add_argument('--max-accel', type=float, default=0.3)
    p.add_argument('--max-ang-accel', type=float, default=0.5)
    p.add_argument('--no-return-home', action='store_true')
    p.add_argument('--cmd-vel-topic', default='/cmd_vel')
    p.add_argument('--motor-power-service', default='/motor_power')
    p.add_argument('--corr-max-d-error', type=float, default=1.0)



class GridMotionNode(TimedNode):
    label = ''               # 'vicon' / 'odom'
    stale_label = ''         # abort message
    stop_message = ''        # warning logged when ctrlc
    pose_msg_type = None

    def setup_pose_source(self):
        """called before any publisher/subscriber is created"""

    def on_pose(self, msg):
        """set self.speed then call store_pose()"""
        raise NotImplementedError

    def waiting_message(self):
        raise NotImplementedError

    def no_pose_message(self):
        raise NotImplementedError

    def zero_odometry(self):
        """for pose that needs to be reset before run (odom)"""

    def __init__(self, args, node_name, pose_topic, pose_timeout):
        super().__init__(node_name)
        self.args = args
        self.pose_timeout = pose_timeout
        self.setup_pose_source()
        self.x = self.y = self.yaw = 0.0
        self.have_pose = False
        self.home = None
        self.intended_yaw = None

        self.plan = deque()      # queue of command dicts
        self.current_cmd = None  # command being executed
        self.cmd_start_time = 0.0
        self.cmd_start_pose = (0.0, 0.0, 0.0)

        self.corr_state = {}
        self.corr_filtered = {}
        self.pose_seq = 0
        self.last_pose_time = None
        self.pose_stamp = None
        self.speed = 0.0         # [m/s]
        self.yaw_rate = 0.0      # [rad/s]
        self.is_finished = False

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.pub = self.create_publisher(TwistStamped, args.cmd_vel_topic, qos)
        self.create_subscription(self.pose_msg_type, pose_topic, self.on_pose, qos)
        self.motor_client = self.create_client(SetBool, args.motor_power_service)
        self.timer_period = 1.0 / self.args.rate
        self.control_timer = None
        self.sender = TwistSender(self, self.pub, args.max_accel, args.max_ang_accel, self.timer_period)

    def store_pose(self, x, y, yaw, stamp):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.have_pose = True
        self.last_pose_time = self.get_time_sec()
        self.pose_stamp = stamp
        self.pose_seq += 1

    def pose(self):
        return self.x, self.y, self.yaw

    def send(self, vx=0.0, vy=0.0, wz=0.0):
        self.sender.send(vx, vy, wz)

    def reset_correction(self):
        self.corr_state.clear()
        self.corr_filtered.clear()

    def tracking_correction(self, axis, error, gain, damping_gain, max_speed, tol):
        now = self.get_time_sec()
        seq = self.pose_seq
        # vicon message stamps
        stamp = self.pose_stamp if self.pose_stamp is not None else now
        prev = self.corr_state.get(axis)
        d_error = 0.0

        if prev is None:
            self.corr_state[axis] = (error, stamp, now, seq)
        else:
            prev_error, prev_stamp, prev_now, prev_seq = prev
            if seq != prev_seq:
                dt = stamp - prev_stamp if stamp > prev_stamp else now - prev_now
                if dt > 1e-6:
                    d_error = (error - prev_error) / dt
                    d_error = clamp(d_error, -self.args.corr_max_d_error, self.args.corr_max_d_error)
                self.corr_state[axis] = (error, stamp, now, seq)

        if abs(error) <= tol:
            raw_target = 0.0
        else:
            raw_target = gain * error + damping_gain * d_error
            raw_target = clamp(raw_target, -max_speed, max_speed)

        prev_filt = self.corr_filtered.get(axis)
        if prev_filt is None or self.args.corr_filter_tau <= 0:
            filtered = raw_target
        else:
            prev_val, prev_t = prev_filt
            dt_f = now - prev_t
            alpha = clamp(dt_f / (self.args.corr_filter_tau + dt_f), 0.0, 1.0)
            filtered = prev_val + (raw_target - prev_val) * alpha

        self.corr_filtered[axis] = (filtered, now)
        return filtered

    def initialize(self):
        # wait for pose, enable motors, save starting pose
        self.get_logger().info(self.waiting_message())
        deadline = self.get_time_sec() + 10.0
        while rclpy.ok() and not self.have_pose and self.get_time_sec() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not self.have_pose:
            raise RuntimeError(self.no_pose_message())

        enabled = call_service(self, self.motor_client, SetBool.Request(data=True), 3.0, 5.0)
        if enabled is None:
            self.get_logger().warn('motor power service not found')
        elif not enabled:
            self.get_logger().warn('motor power enable failed or timed out')

        self.zero_odometry()

        self.home = self.pose()
        self.intended_yaw = self.home[2]
        self.get_logger().info(f'home: x={self.home[0]:.3f} y={self.home[1]:.3f} '
                               f'yaw={math.degrees(self.home[2]):+.2f} deg')


    def plan_wait(self, duration):
        self.plan.append({'type': 'wait', 'duration': duration})

    def plan_turn(self, target_yaw):
        self.plan.append({'type': 'turn', 'target_yaw': target_yaw})

    def plan_turn_by(self, angle):
        if self.intended_yaw is None:
            _, _, self.intended_yaw = self.pose()
        self.intended_yaw = normalize(self.intended_yaw + angle)
        self.plan.append({'type': 'turn', 'target_yaw': self.intended_yaw, 'spin': 1.0 if angle >= 0 else -1.0})

    def plan_drive(self, distance):
        self.plan.append({'type': 'drive', 'distance': distance})

    def plan_move_to(self, target_x, target_y):
        self.plan.append({'type': 'move_to', 'target_x': target_x, 'target_y': target_y})

    def generate_grid_plan(self):
        self.plan.clear()
        rows = self.args.grid_size
        cols = self.args.grid_cols if self.args.grid_cols is not None else self.args.grid_size
        csx = self.args.cell_size_x if self.args.cell_size_x is not None else self.args.cell_size
        csy = self.args.cell_size_y if self.args.cell_size_y is not None else self.args.cell_size
        self.get_logger().info(f'Generating {rows} rows x {cols} cols grid plan ({self.args.platform})')

        self.plan_wait(0.3)

        if self.args.platform == 'omni':
            hx, hy, _ = self.home
            waypoints = []
            for row in range(rows):
                wy = hy + row * csy
                col_order = range(cols) if row % 2 == 0 else reversed(range(cols))
                for col in col_order:
                    waypoints.append((hx + col * csx, wy))
            for wx, wy in waypoints[1:]:
                self.plan_move_to(wx, wy)
        else:
            for row in range(rows):
                for _ in range(cols - 1):
                    self.plan_drive(csx)
                if row < rows - 1:
                    turn = -math.pi / 2 if row % 2 == 0 else math.pi / 2
                    self.plan_turn_by(turn)
                    self.plan_drive(csy)
                    self.plan_turn_by(turn)

        if not self.args.no_return_home:
            hx, hy, hyaw = self.home
            if self.args.platform == 'omni':
                self.plan_move_to(hx, hy)
                self.plan_turn(hyaw)
            else:
                self.plan.append({'type': 'return_home_diff'})

        self.plan_wait(0.8)


    def start_control_loop(self):
        self.generate_grid_plan()
        self.control_timer = self.create_timer(self.timer_period, self.control_tick)

    def control_tick(self):
        if self.is_finished:
            return

        now = self.get_time_sec()

        if self.last_pose_time is not None and now - self.last_pose_time > self.pose_timeout:
            self.get_logger().error(f'{self.stale_label} stale for {now - self.last_pose_time:.1f}s, aborting')
            self.send(0.0, 0.0, 0.0)
            self.is_finished = True
            return

        # if no current command get next from plan
        if self.current_cmd is None:
            if not self.plan:
                fx, fy, fyaw = self.pose()
                self.get_logger().info(
                    f'final at ({fx:+.3f}, {fy:+.3f}) yaw {math.degrees(fyaw):+.1f} deg; '
                    f'dx={fx - self.home[0]:+.3f}m dy={fy - self.home[1]:+.3f}m '
                    f'dyaw={math.degrees(normalize(fyaw - self.home[2])):+.2f} deg')
                self.get_logger().info('Complete!')
                self.send(0.0, 0.0, 0.0)
                self.is_finished = True
                return

            self.current_cmd = self.plan.popleft()
            self.cmd_start_time = now
            self.cmd_start_pose = self.pose()
            self.reset_correction()
            self.get_logger().info(f"executing {self.current_cmd['type']}")

        # execute current command
        cmd = self.current_cmd
        ctype = cmd['type']
        elapsed = now - self.cmd_start_time
        x, y, yaw = self.pose()
        start_x, start_y, start_yaw = self.cmd_start_pose
        a = self.args

        if ctype == 'wait':
            self.send(0.0, 0.0, 0.0)
            if elapsed >= cmd['duration']:
                self.current_cmd = None

        elif ctype == 'turn':
            raw_angle = cmd['target_yaw'] - start_yaw
            spin = cmd.get('spin', 0.0)
            if spin > 0 and raw_angle < 0:
                raw_angle += 2 * math.pi
            elif spin < 0 and raw_angle > 0:
                raw_angle -= 2 * math.pi
            angle = raw_angle if spin else normalize(raw_angle)

            if not cmd.get('logged'):
                cmd['logged'] = True
                self.get_logger().info(
                    f"turn cmd {math.degrees(angle):+.1f} deg (start yaw {math.degrees(start_yaw):+.1f}) "
                    f"-> target {self.label} yaw {math.degrees(normalize(cmd['target_yaw'])):+.1f} deg")

            T = duration_for_peak(angle, a.turn_speed)
            if T <= 0:
                self.current_cmd = None
                return

            homing = elapsed >= T
            tau = min(elapsed / T, 1.0)
            w_ff = 0.0 if homing else (angle / T) * velocity_factor(tau)

            turned = normalize(yaw - start_yaw)
            if abs(angle) > math.pi:
                if spin > 0 and turned < 0:
                    turned += 2 * math.pi
                elif spin < 0 and turned > 0:
                    turned -= 2 * math.pi

            expected = angle * position_factor(tau)
            error = expected - turned if abs(angle) > math.pi else normalize(expected - turned)

            w_correction = self.tracking_correction('turn', error, a.corr_gain, a.corr_damping_gain,
                                                    a.turn_speed, a.angle_tol)
            self.send(wz=w_ff + w_correction)

            if homing and abs(error) <= a.angle_tol and abs(self.yaw_rate) < 0.05:
                self.get_logger().info(f"landed {math.degrees(normalize(cmd['target_yaw'] - yaw)):+.2f} deg off")
                self.send(0.0, 0.0, 0.0)
                self.current_cmd = None
            elif homing and elapsed >= T + a.homing_timeout:
                self.get_logger().warn(f'turn residual {math.degrees(error):.2f} deg at timeout')
                self.current_cmd = None

        elif ctype == 'drive':
            dist = cmd['distance']
            T = duration_for_peak(dist, a.max_speed)
            if T <= 0:
                self.current_cmd = None
                return

            if not cmd.get('logged'):
                cmd['logged'] = True
                tgt_x = start_x + dist * math.cos(start_yaw)
                tgt_y = start_y + dist * math.sin(start_yaw)
                self.get_logger().info(
                    f"drive cmd {dist:+.3f}m (start yaw {math.degrees(start_yaw):+.1f}) "
                    f"{self.label} ({start_x:+.3f}, {start_y:+.3f}) -> ({tgt_x:+.3f}, {tgt_y:+.3f})")

            homing = elapsed >= T
            tau = min(elapsed / T, 1.0)
            v_ff = 0.0 if homing else (dist / T) * velocity_factor(tau)

            dx = x - start_x
            dy = y - start_y
            travelled = dx * math.cos(start_yaw) + dy * math.sin(start_yaw)
            lateral = -dx * math.sin(start_yaw) + dy * math.cos(start_yaw)

            expected = dist * position_factor(tau)
            error = expected - travelled

            v_correction = self.tracking_correction('drive', error, a.corr_gain, a.corr_damping_gain,
                                                    a.max_speed, a.position_tol)
            heading_fix = clamp(HEADING_GAIN*normalize(start_yaw-yaw-
                          clamp(LATERAL_GAIN*lateral,-LATERAL_CLAMP, LATERAL_CLAMP)),-HEADING_FIX_LIMIT * a.turn_speed,
                                HEADING_FIX_LIMIT * a.turn_speed)
            self.send(vx=v_ff + v_correction, wz=heading_fix)

            if homing and abs(error) <= a.position_tol and abs(self.speed) < 0.02:
                self.get_logger().info(f"landed {error:+.3f}m along, {lateral:+.3f}m lateral off")
                self.send(0.0, 0.0, 0.0)
                self.current_cmd = None
            elif elapsed >= T + a.homing_timeout:
                self.get_logger().warn(f'drive residual {error:.3f}m at timeout')
                self.current_cmd = None

        elif ctype == 'move_to':
            if not cmd.get('logged'):
                cmd['logged'] = True
                self.get_logger().info(
                    f"move_to {self.label} ({start_x:+.3f}, {start_y:+.3f}) -> "
                    f"({cmd['target_x']:+.3f}, {cmd['target_y']:+.3f})")
            dx = cmd['target_x'] - start_x
            dy = cmd['target_y'] - start_y
            T = max(duration_for_peak(dx, a.max_speed), duration_for_peak(dy, a.max_speed))
            if T <= 0:
                self.current_cmd = None
                return

            homing = elapsed >= T
            tau = min(elapsed / T, 1.0)
            vf = 0.0 if homing else velocity_factor(tau)
            pf = position_factor(tau)

            expected_x, expected_y = start_x + dx * pf, start_y + dy * pf
            err_x, err_y = expected_x - x, expected_y - y

            vx_corr = self.tracking_correction('move_x', err_x, a.corr_gain, a.corr_damping_gain,
                                               a.max_speed, a.position_tol)
            vy_corr = self.tracking_correction('move_y', err_y, a.corr_gain, a.corr_damping_gain,
                                               a.max_speed, a.position_tol)

            vx_world = (dx / T) * vf + vx_corr
            vy_world = (dy / T) * vf + vy_corr
            vx_body, vy_body = world_to_body(vx_world, vy_world, yaw)
            heading_fix = clamp(HEADING_GAIN * normalize(start_yaw - yaw),
                                -HEADING_FIX_LIMIT * a.turn_speed, HEADING_FIX_LIMIT * a.turn_speed)

            self.send(vx=vx_body, vy=vy_body, wz=heading_fix)

            if homing and math.hypot(err_x, err_y) <= a.position_tol and abs(self.speed) < 0.02:
                self.send(0.0, 0.0, 0.0)
                self.current_cmd = None
            elif elapsed >= T + a.homing_timeout:
                self.get_logger().warn(f'move_to residual {math.hypot(err_x, err_y):.3f}m at timeout')
                self.current_cmd = None

        elif ctype == 'return_home_diff':
            hx, hy, hyaw = self.home
            dist = math.hypot(hx - x, hy - y)
            retries = cmd.get('retries', 0)
            if dist > a.return_position_tol and retries < a.return_home_max_retries:
                if not cmd.get('aimed'):
                    # turn then reevaluate
                    target_angle = math.atan2(hy - y, hx - x)
                    self.get_logger().info(
                        f"return home: {self.label} ({x:+.3f}, {y:+.3f}) -> home ({hx:+.3f}, {hy:+.3f}) "
                        f"dist {dist:.3f}m angle {math.degrees(target_angle):+.1f} deg")
                    cmd['aimed'] = True
                    self.plan.appendleft(cmd)  # requeued
                    self.plan.appendleft({'type': 'turn', 'target_yaw': target_angle})
                else:
                    # distance measured after the turn
                    self.get_logger().info(
                        f"return home attempt {retries + 1}: drive {dist:.3f}m toward home ({hx:+.3f}, {hy:+.3f})")
                    cmd['aimed'] = False
                    cmd['retries'] = retries + 1
                    self.plan.appendleft(cmd)
                    self.plan.appendleft({'type': 'drive', 'distance': dist})
            else:
                if dist > a.return_position_tol:
                    self.get_logger().warn(f'return_home_diff: giving up after {retries} attempts,'
                                           f'residual distance {dist:.3f}m')
                self.plan.appendleft({'type': 'turn', 'target_yaw': hyaw})
            self.current_cmd = None


def run_grid(args, node_class):
    # create the node, run path, finish with zero-velocity burst
    init_ros()
    node = node_class(args)
    stop_on_signals()

    try:
        node.initialize()
        node.start_control_loop()
        while rclpy.ok() and not node.is_finished:
            rclpy.spin_once(node, timeout_sec=0.1)

    except KeyboardInterrupt:
        node.get_logger().warn(node.stop_message)
    except Exception as exc:
        node.get_logger().error(f'{exc}')
        raise
    finally:
        ignore_stop_signals()
        node.sender.reset()
        send_zero_burst(node, 10, 0.02)
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()