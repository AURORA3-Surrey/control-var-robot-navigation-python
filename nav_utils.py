# helpers shared by the navigation scripts

import json
import math
import os
import signal
import sys
import time
import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node


def yaw_from_quaternion(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def normalize(angle):
    # wrap angle to [-pi, pi]
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def clamp(v, low, high):
    return max(low, min(high, v))


DEFAULT_YAW_OFFSET_FILE = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), 'vicon_yaw_offset.json')

def add_yaw_offset_arguments(parser, offset_flag):
    parser.add_argument(offset_flag, type=float, default=None)
    parser.add_argument('--yaw-offset-file', default=DEFAULT_YAW_OFFSET_FILE)
    parser.add_argument('--no-yaw-offset-file', action='store_true')
    parser.add_argument('--allow-stale-yaw-offset', action='store_true')


def load_yaw_offset_file(path):
    try:
        with open(path, 'r') as f:
            data = json.load(f)
        created = str(data.get('created', ''))[:10] or 'unknown'
        off = float(data['offset_deg'])
        if not -360.0 <= off <= 360.0:
            return None, created
        return off, created
    except Exception:
        return None, 'unknown'


def read_yaw_offset(args, cli_offset_deg):
    # return (offset_deg, origin, refusal reason or '')
    if cli_offset_deg is not None:
        return cli_offset_deg, 'command line', ''
    if args.no_yaw_offset_file:
        return 0.0, 'default 0 (--no-yaw-offset-file)', ''
    path = args.yaw_offset_file
    if not os.path.isfile(path):
        return (0.0, f'default 0, no offset calibration file ({path})',
                f'no offset calibration file ({path})')
    offset, created = load_yaw_offset_file(path)
    if offset is None:
        return (0.0, f'default 0, unreadable calibration file ({path})',
                f'unreadable offset calibration file ({path})')
    today = time.strftime('%Y-%m-%d')
    stale = '' if created == today else f'offset calibration dated {created} ({today})'
    return offset, f'calibration file: {path} (dated {created})', stale


def require_fresh_yaw_offset(args, cli_offset_deg):
    refusal = read_yaw_offset(args, cli_offset_deg)[2]
    if refusal and not args.allow_stale_yaw_offset:
        print(f'{refusal}')
        print('python3 vicon_yaw_calibration.py then re-run this script')
        print('(bypass with --allow-stale-yaw-offset, --vicon-yaw-offset-deg, '
              'or --no-yaw-offset-file)')
        sys.exit(1)


def resolve_yaw_offset(node, args, cli_offset_deg):
    offset, source, _ = read_yaw_offset(args, cli_offset_deg)
    node.get_logger().info(f'vicon yaw offset: {offset:+.1f} deg ({source})')
    return offset


class VelocityFilter:
    # lowpass filtered speed and yaw rate from finite differences of (x, y, yaw, stamp)

    def __init__(self, tau):
        self.tau = tau
        self.prev = None
        self.v = 0.0
        self.wz = 0.0

    def update(self, x, y, yaw, stamp):
        prev = self.prev
        if prev is not None and 0.0 < stamp - prev[3] < 0.2:
            dt = stamp - prev[3]
            v_raw = math.hypot(x - prev[0], y - prev[1]) / dt
            w_raw = normalize(yaw - prev[2]) / dt
            alpha = dt / (self.tau + dt)
            self.v = alpha * v_raw + (1.0 - alpha) * self.v
            self.wz = alpha * w_raw + (1.0 - alpha) * self.wz
        else:
            self.v = 0.0
            self.wz = 0.0
        self.prev = (x, y, yaw, stamp)


def init_ros():
    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    except (ImportError, TypeError):
        rclpy.init()


def raise_keyboard_interrupt(signum, frame):
    raise KeyboardInterrupt


def stop_on_signals():
    # turn into KeyboardInterrupt so the shutdown code in finally runs
    signal.signal(signal.SIGINT, raise_keyboard_interrupt)
    signal.signal(signal.SIGTERM, raise_keyboard_interrupt)


def ignore_stop_signals():
    # used while stopping the robot (second ctrl-c cannot interrupt the stop)
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    signal.signal(signal.SIGTERM, signal.SIG_IGN)


class TimedNode(Node):
    def get_time_sec(self):
        return self.get_clock().now().nanoseconds / 1e9


def call_service(node, client, request, wait_timeout, call_timeout):
    if not client.wait_for_service(timeout_sec=wait_timeout):
        return None
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=call_timeout)
    return bool(future.done() and future.result() is not None and future.result().success)


class TwistSender:
    # publishes TwistStamped and limits how fast velocity can change

    def __init__(self, node, publisher, max_accel, max_ang_accel, period):
        self.node = node
        self.pub = publisher
        self.max_accel = max_accel
        self.max_ang_accel = max_ang_accel
        self.period = period
        self.last_vx = self.last_vy = self.last_wz = 0.0
        self.last_send_time = None

    def reset(self):
        # used for stopping
        self.last_vx = self.last_vy = self.last_wz = 0.0

    def send(self, vx=0.0, vy=0.0, wz=0.0):
        now = self.node.get_time_sec()
        dt = (now - self.last_send_time) if self.last_send_time else self.period
        self.last_send_time = now
        max_dv = self.max_accel * dt
        max_dw = self.max_ang_accel * dt
        vx = clamp(vx, self.last_vx - max_dv, self.last_vx + max_dv)
        if vy is not None:
            vy = clamp(vy, self.last_vy - max_dv, self.last_vy + max_dv)
            self.last_vy = vy
        wz = clamp(wz, self.last_wz - max_dw, self.last_wz + max_dw)
        self.last_vx, self.last_wz = vx, wz

        msg = TwistStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.twist.linear.x = float(vx)
        if vy is not None:
            msg.twist.linear.y = float(vy)
        msg.twist.angular.z = float(wz)
        self.pub.publish(msg)


def send_zero_burst(node, count, spin_timeout):
    sent = 0
    for _ in range(count):
        try:
            node.send()
            sent += 1
            rclpy.spin_once(node, timeout_sec=spin_timeout)
        except Exception:
            break
        time.sleep(0.02)
    node.get_logger().info(f'stop {sent}/{count} zero commands published')
