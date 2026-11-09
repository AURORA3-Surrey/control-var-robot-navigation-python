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
from grid_commands import Drive, MoveTo, ReturnHomeDiff, Turn, Wait


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

        self.plan = deque()      # queue of command objects
        self.current_cmd = None  # command being executed

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
        self.plan.append(Wait(duration))

    def plan_turn(self, target_yaw):
        self.plan.append(Turn(target_yaw))

    def plan_turn_by(self, angle):
        if self.intended_yaw is None:
            _, _, self.intended_yaw = self.pose()
        self.intended_yaw = normalize(self.intended_yaw + angle)
        self.plan.append(Turn(self.intended_yaw, spin=1.0 if angle >= 0 else -1.0))

    def plan_drive(self, distance):
        self.plan.append(Drive(distance))

    def plan_move_to(self, target_x, target_y):
        self.plan.append(MoveTo(target_x, target_y))

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
                self.plan.append(ReturnHomeDiff())

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
            self.current_cmd.start_time = now
            self.current_cmd.start_pose = self.pose()
            self.reset_correction()
            self.get_logger().info(f"executing {self.current_cmd['type']}")

        if self.current_cmd.step(self, now):
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