import argparse
import json
import math
import sys
import time
import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_srvs.srv import SetBool

from nav_utils import (DEFAULT_YAW_OFFSET_FILE, TimedNode, TwistSender, call_service, clamp,
                       ignore_stop_signals, init_ros, normalize, send_zero_burst, stop_on_signals,
                       yaw_from_quaternion)

class SafetyAbort(RuntimeError):
    pass

class YawCalibNode(TimedNode):
    # measures the yaw offset by moving forward and watching which world direction the robot moves
    # (measured offset only used for the return trip)

    def __init__(self, args):
        super().__init__('vicon_yaw_calibration')
        self.a = args
        self.x = self.y = self.yaw = 0.0
        self.have_pose = False
        self.last_pose_time = None
        self.off_rad = 0.0  # measured offset set after movement

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.pub = self.create_publisher(TwistStamped, args.cmd_vel_topic, qos)
        self.create_subscription(PoseStamped, args.vicon_topic, self.vicon_cb, qos)
        self.motor_client = self.create_client(SetBool, args.motor_power_service)
        self.timer_period = 1.0 / self.a.rate
        self.sender = TwistSender(
            self, self.pub, args.accel_lin, args.accel_ang, self.timer_period)

    def vicon_cb(self, msg):
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        self.yaw = yaw_from_quaternion(msg.pose.orientation)  # no offset
        self.have_pose = True
        self.last_pose_time = time.time()

    def pose(self):
        return self.x, self.y, self.yaw

    def send(self, vx=0.0, wz=0.0):
        self.sender.send(vx, None, wz)

    def spin_tick(self):
        rclpy.spin_once(self, timeout_sec=0.005)
        if (self.last_pose_time is not None
                and time.time() - self.last_pose_time > self.a.vicon_timeout):
            raise SafetyAbort(f'vicon stream stale for {time.time() - self.last_pose_time:.1f}s')


def wait_for_pose(node, timeout=10.0):
    node.get_logger().info(f'waiting for {node.a.vicon_topic}')
    t0 = time.time()
    while not node.have_pose:
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.time() - t0 > timeout:
            raise SafetyAbort('no vicon pose received')
    node.get_logger().info('vicon stream live')


def wait_stationary(node, timeout=10.0):
    # start pose must be still
    node.get_logger().info('waiting for robot to be stationary')
    t0 = time.time()
    while time.time() - t0 < timeout:
        samples = []
        while len(samples) < 15 and time.time() - t0 < timeout:
            node.spin_tick()
            samples.append(node.pose())
            time.sleep(0.03)
        xs = [s[0] for s in samples]
        ys = [s[1] for s in samples]
        spread = max(max(xs) - min(xs), max(ys) - min(ys))
        if spread < 0.005:
            return
    raise SafetyAbort('robot not stationary aborting')


def set_motor_power(node, on):
    ok = call_service(node, node.motor_client, SetBool.Request(data=on), 3.0, 3.0)
    if ok is None:
        node.get_logger().warning('motor_power service not available continuing')
        return False
    node.get_logger().info(f'motor power {"ON" if on else "OFF"} {ok}')
    return ok


def move_forward(node):
    a = node.a
    x0, y0, yaw0 = node.pose()
    node.get_logger().info(
        f'start pose: ({x0:+.3f}, {y0:+.3f}) raw vicon yaw {math.degrees(yaw0):+.1f} deg')
    node.get_logger().info(
        f'forward movement {a.movement_distance:.2f} m at {a.movement_speed:.2f} m/s')
    t0 = time.time()
    while time.time() - t0 < a.movement_timeout:
        node.spin_tick()
        x, y, yaw = node.pose()
        disp = math.hypot(x - x0, y - y0)
        if disp >= a.movement_distance:
            break
        if disp > a.max_displacement:
            raise SafetyAbort(f'displacement runaway: {disp:.3f} m')
        wz = clamp(1.5 * normalize(yaw0 - yaw), -0.3, 0.3)
        node.send(vx=a.movement_speed, wz=wz)
        time.sleep(1.0 / a.rate)
    node.send(0.0, 0.0)
    x, y, yaw = node.pose()
    disp = math.hypot(x - x0, y - y0)
    if disp < a.min_displacement:
        raise SafetyAbort(
            f'robot moved only {disp:.3f} m (< {a.min_displacement:.2f} m min) motors enabled?')
    return x0, y0, yaw0, x, y, yaw, disp


def measure_offset(x0, y0, yaw0, x, y, yaw_end):
    bearing = math.atan2(y - y0, x - x0)
    yaw_mean = yaw0 + 0.5 * normalize(yaw_end - yaw0)
    off = math.degrees(normalize(bearing - yaw_mean))
    snap = round(off / 90.0) * 90.0
    snap = ((snap + 180.0) % 360.0) - 180.0  # [-180, 180]
    if snap == -180.0:
        snap = 180.0
    residual = math.degrees(normalize(math.radians(off - snap)))
    return bearing, yaw_mean, off, snap, residual


def return_home(node, x0, y0, yaw0):
    # drive back to the start pose using the measured offset (+ restore original heading)
    a = node.a
    off = node.off_rad
    node.get_logger().info('turn toward start, drive back, restore heading')

    t0 = time.time()
    while time.time() - t0 < a.return_timeout:
        node.spin_tick()
        x, y, yaw = node.pose()
        dist = math.hypot(x0 - x, y0 - y)
        if dist <= a.home_tol:
            break
        err = normalize(math.atan2(y0 - y, x0 - x) - (yaw + off))
        v = clamp(1.5 * dist, 0.0, a.return_speed) if abs(err) <= math.radians(30.0) else 0.0
        node.send(vx=v, wz=clamp(2.0 * err, -a.turn_speed, a.turn_speed))
        time.sleep(1.0 / a.rate)
    node.send(0.0, 0.0)

    # restore original heading
    t0 = time.time()
    while time.time() - t0 < a.turn_timeout:
        node.spin_tick()
        _, _, yaw = node.pose()
        err = normalize(yaw0 - yaw)
        if abs(err) <= math.radians(a.turn_tol_deg):
            break
        node.send(wz=clamp(2.0 * err, -a.turn_speed, a.turn_speed))
        time.sleep(1.0 / a.rate)
    node.send(0.0, 0.0)
    return node.pose()


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument('--vicon-topic', default='/vicon/Dingo/Dingo')
    p.add_argument('--cmd-vel-topic', default='/cmd_vel')
    p.add_argument('--motor-power-service', default='/motor_power')
    p.add_argument('--movement-speed', type=float, default=0.05)
    p.add_argument('--movement-distance', type=float, default=0.12)
    p.add_argument('--min-displacement', type=float, default=0.03)
    p.add_argument('--max-displacement', type=float, default=0.30)
    p.add_argument('--movement-timeout', type=float, default=10.0)
    p.add_argument('--return-speed', type=float, default=0.06)
    p.add_argument('--turn-speed', type=float, default=0.4)
    p.add_argument('--turn-tol-deg', type=float, default=2.0)
    p.add_argument('--return-timeout', type=float, default=25.0)
    p.add_argument('--turn-timeout', type=float, default=8.0)
    p.add_argument('--home-tol', type=float, default=0.015)
    p.add_argument('--vicon-timeout', type=float, default=0.5)
    p.add_argument('--rate', type=float, default=25.0)
    p.add_argument('--accel-lin', type=float, default=0.3)
    p.add_argument('--accel-ang', type=float, default=1.0)
    p.add_argument('--keep-motors-on', action='store_true')
    p.add_argument('--no-save', action='store_true')
    p.add_argument('--save-file', default=DEFAULT_YAW_OFFSET_FILE)
    return p.parse_args()


def main():
    args = parse_args()
    init_ros()
    node = YawCalibNode(args)
    stop_on_signals()

    exit_code = 1
    try:
        wait_for_pose(node)
        wait_stationary(node)
        set_motor_power(node, True)
        send_zero_burst(node, 5, 0.01)

        x0, y0, yaw0, xe, ye, yawe, disp = move_forward(node)
        bearing, yaw_mean, off_deg, snap, residual = measure_offset(x0, y0, yaw0, xe, ye, yawe)
        node.off_rad = math.radians(off_deg)

        node.get_logger().info(
            f'movement: {disp:.3f} m, world bearing {math.degrees(bearing):+.1f} deg, '
            f'raw vicon yaw mean {math.degrees(yaw_mean):+.1f} deg')
        node.get_logger().info(
            f'measured yaw offset: {off_deg:+.1f} deg snapped {snap:+.0f} deg '
            f'(residual {residual:+.1f} deg)')
        if abs(residual) > 20.0:
            node.get_logger().warning('residual > 20 deg: movement was not clean')

        if snap == 0.0:
            node.get_logger().info('vicon +x matches robot forward (no offset needed)')
        elif snap == 180.0:
            node.get_logger().info('vicon +x faces the robot rear')
        else:
            node.get_logger().warning(
                'rigid body is rotated around 90 deg, recreate the rigid body in Tracker')

        node.get_logger().info(f'recommended --vicon-yaw-offset-deg {snap:+.0f}')
        node.get_logger().info(f'yaw_offset_result: {snap:+.0f}')

        fx, fy, fyaw = return_home(node, x0, y0, yaw0)
        dx, dy = fx - x0, fy - y0
        dyaw = math.degrees(normalize(fyaw - yaw0))
        dist = math.hypot(dx, dy)
        node.get_logger().info(
            f'return at ({fx:+.3f}, {fy:+.3f}) | dx={dx:+.3f}m dy={dy:+.3f}m '
            f'dyaw={dyaw:+.2f} deg')
        returned = dist <= 0.04 and abs(dyaw) <= 5.0
        if returned:
            node.get_logger().info('return ok measured offset validated')
        else:
            node.get_logger().warning(
                'return imperfect reposition the robot by hand (measurement still valid)')

        if not args.no_save:
            payload = {
                'offset_deg': snap,
                'measured_deg': round(off_deg, 1),
                'snap_residual_deg': round(residual, 1),
                'vicon_topic': args.vicon_topic,
                'created': time.strftime('%Y-%m-%d %H:%M:%S'),
            }
            with open(args.save_file, 'w') as f:
                json.dump(payload, f, indent=2)
            node.get_logger().info(
                f'saved: {args.save_file} (vicon_grid.py auto-load)')

        exit_code = 0 if returned else 2

    except SafetyAbort as exc:
        node.get_logger().error(f'{exc}')
        exit_code = 1
    except KeyboardInterrupt:
        node.get_logger().warning('stopping robot')
        exit_code = 1
    except Exception as exc:
        node.get_logger().error(f'{exc}')
        raise
    finally:
        ignore_stop_signals()
        node.sender.reset()
        send_zero_burst(node, 10, 0.01)
        if not args.keep_motors_on:
            try:
                set_motor_power(node, False)
            except Exception:
                pass
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()