# grid runner shared by vicon_grid and test_grid (wheel odom)
# initialize(), then start_control_loop() plans grid, control_tick() runs one command at a time

import math
from collections import deque
import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_srvs.srv import SetBool
from nav_utils import (TimedNode, TwistSender, call_service, ignore_stop_signals, init_ros, normalize,
                       send_zero_burst, stop_on_signals)
from grid_commands import Drive, MoveTo, ReturnHomeDiff, TrackingCorrection, Turn, Wait


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


def grid_dimensions(args):
    # (rows, cols, cell size along x, cell size along y)
    rows = args.grid_size
    cols = args.grid_cols if args.grid_cols is not None else args.grid_size
    csx = args.cell_size_x if args.cell_size_x is not None else args.cell_size
    csy = args.cell_size_y if args.cell_size_y is not None else args.cell_size
    return rows, cols, csx, csy


def build_grid_plan(args, home):
    # path over the grid starting at home = (x, y, yaw) then back
    rows, cols, csx, csy = grid_dimensions(args)
    hx, hy, hyaw = home
    omni = args.platform == 'omni'
    plan = [Wait(0.3)]

    if omni:
        waypoints = []
        for row in range(rows):
            wy = hy + row * csy
            col_order = range(cols) if row % 2 == 0 else reversed(range(cols))
            for col in col_order:
                waypoints.append((hx + col * csx, wy))
        for wx, wy in waypoints[1:]:
            plan.append(MoveTo(wx, wy))
    else:
        intended_yaw = hyaw  # accumulates ideal angle so turn errors don't add up
        for row in range(rows):
            for _ in range(cols - 1):
                plan.append(Drive(csx))
            if row < rows - 1:  # turn, step sideways, turn
                turn = -math.pi / 2 if row % 2 == 0 else math.pi / 2
                spin = 1.0 if turn >= 0 else -1.0
                intended_yaw = normalize(intended_yaw + turn)
                plan.append(Turn(intended_yaw, spin))
                plan.append(Drive(csy))
                intended_yaw = normalize(intended_yaw + turn)
                plan.append(Turn(intended_yaw, spin))

    if not args.no_return_home:
        if omni:
            plan.append(MoveTo(hx, hy))
            plan.append(Turn(hyaw))
        else:
            plan.append(ReturnHomeDiff())

    plan.append(Wait(0.8))
    return plan


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

        self.plan = deque()      # queue of command objects
        self.current_cmd = None  # command being executed

        # filled by store_pose()
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
        self.correction = TrackingCorrection(self.get_time_sec, args)

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

    def correct(self, axis, error, max_speed, tol):
        return self.correction.compute(axis, error, max_speed, tol, self.pose_seq, self.pose_stamp)

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
        self.get_logger().info(f'home: x={self.home[0]:.3f} y={self.home[1]:.3f} '
                               f'yaw={math.degrees(self.home[2]):+.2f} deg')

    def start_control_loop(self):
        rows, cols, _, _ = grid_dimensions(self.args)
        self.get_logger().info(f'Generating {rows} rows x {cols} cols grid plan ({self.args.platform})...')
        self.plan.clear()
        self.plan.extend(build_grid_plan(self.args, self.home))
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
            self.correction.reset()
            self.get_logger().info(f"executing {self.current_cmd.name}")

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