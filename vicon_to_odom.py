import argparse
import math
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav_utils import (TimedNode, VelocityFilter, add_yaw_offset_arguments, init_ros, normalize,
                       require_fresh_yaw_offset, resolve_yaw_offset, stop_on_signals, yaw_from_quaternion)


class ViconToOdom(TimedNode):
    def __init__(self, args):
        super().__init__('vicon_to_odom')
        self.a = args
        self.yaw_offset_deg = resolve_yaw_offset(self, args, args.yaw_offset_deg)
        self.velocity = VelocityFilter(args.vel_filter_tau)
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(PoseStamped, args.vicon_topic, self.vicon_cb, qos)
        self.pub = self.create_publisher(Odometry, args.odom_topic, qos)
        self.get_logger().info(f'{args.vicon_topic} -> {args.odom_topic}')

    def vicon_cb(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        yaw = yaw_from_quaternion(msg.pose.orientation)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.velocity.update(x, y, yaw, stamp)

        # optional rigid-body alignment (uniform scale + yaw offset about world Z)
        s = self.a.scale
        off = math.radians(self.yaw_offset_deg)
        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = msg.header.frame_id
        out.child_frame_id = 'base_footprint'
        if off != 0.0:
            c, si = math.cos(off), math.sin(off)
            out.pose.pose.position.x = s * (c * x - si * y)
            out.pose.pose.position.y = s * (si * x + c * y)
            yaw_out = normalize(yaw + off)
            out.pose.pose.orientation.z = math.sin(yaw_out / 2.0)
            out.pose.pose.orientation.w = math.cos(yaw_out / 2.0)
        else:
            out.pose.pose.position.x = s * x
            out.pose.pose.position.y = s * y
            out.pose.pose.orientation = msg.pose.orientation
        out.twist.twist.linear.x = self.velocity.v
        out.twist.twist.angular.z = self.velocity.wz
        self.pub.publish(out)


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument('--vicon-topic', default='/vicon/Turtlebot3/Turtlebot3')
    p.add_argument('--odom-topic', default='/vicon/odom')
    p.add_argument('--vel-filter-tau', type=float, default=0.08)
    add_yaw_offset_arguments(p, '--yaw-offset-deg')
    p.add_argument('--scale', type=float, default=1.0)
    return p.parse_args()


def main():
    args = parse_args()
    require_fresh_yaw_offset(args, args.yaw_offset_deg)
    init_ros()
    node = ViconToOdom(args)
    stop_on_signals()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()