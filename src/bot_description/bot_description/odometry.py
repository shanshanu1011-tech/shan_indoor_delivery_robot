#!/usr/bin/env python3

import math
import serial

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState


class DiffDriveNode(Node):
    def __init__(self):
        super().__init__("diff_drive_serial")

        # Parameters
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("wheel_radius", 0.05)
        self.declare_parameter("wheel_base", 0.3)
        self.declare_parameter("ticks_per_rev", 420)

        self.port = self.get_parameter("port").value
        self.baud = self.get_parameter("baud").value
        self.wheel_radius = self.get_parameter("wheel_radius").value
        self.wheel_base = self.get_parameter("wheel_base").value
        self.ticks_per_rev = self.get_parameter("ticks_per_rev").value

        # Serial
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=1)
            self.get_logger().info(f"Opened serial port {self.port} at {self.baud} baud")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            raise

        # Subscriber
        self.create_subscription(TwistStamped, "/cmd_vel", self.cmd_callback, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)

        # State
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.last_left_ticks = 0
        self.last_right_ticks = 0
        self.last_time = self.get_clock().now()

        # Timer
        self.timer = self.create_timer(0.01, self.update)  # 50 Hz

    # -------------------------
    # CMD VEL CALLBACK
    # -------------------------
    def cmd_callback(self, msg: TwistStamped):
        v = msg.twist.linear.x
        w = msg.twist.angular.z

        L = self.wheel_base

        v_l = (int)((51/0.22) * (v - (w * L / 2.0)))
        v_r = (int)((51/0.22) * (v + (w * L / 2.0)))
        # print(v_l," ",v_r)
        cmd = f"o {v_l} {v_r}\n"
        try:
            self.ser.write(cmd.encode())
        except Exception as e:
            self.get_logger().warning(f"Serial write failed: {e}")

    # -------------------------
    # READ ENCODERS
    # -------------------------
    def read_encoders(self):
        try:
            # request encoder values from Arduino
            self.ser.write(b"e\n")

            # read reply
            line = self.ser.readline().decode(errors="ignore").strip()

            parts = line.split()
            if len(parts) == 2:
                l, r = parts
                return int(l), int(r)

        except Exception as e:
            self.get_logger().warning(f"Serial error: {e}")

        return None
    # -------------------------
    # QUATERNION FROM YAW
    # -------------------------
    def quaternion_from_yaw(self, yaw):
        qx = 0.0
        qy = 0.0
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        return qx, qy, qz, qw

    # -------------------------
    # ODOMETRY UPDATE
    # -------------------------
    def update(self):
        data = self.read_encoders()
        if data is None:
            return

        left_ticks, right_ticks = data

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        if dt <= 0.0:
            return

        # Tick differences
        dl = left_ticks - self.last_left_ticks
        dr = right_ticks - self.last_right_ticks

        self.last_left_ticks = left_ticks
        self.last_right_ticks = right_ticks

        # Distance per tick
        dist_per_tick = 2.0 * math.pi * self.wheel_radius / self.ticks_per_rev

        dL = dl * dist_per_tick
        dR = dr * dist_per_tick

        d_center = (dL + dR) / 2.0
        d_theta = (dR - dL) / self.wheel_base

        # Pose update
        self.x += d_center * math.cos(self.theta)
        self.y += d_center * math.sin(self.theta)
        self.theta += d_theta

        vx = d_center / dt
        vth = d_theta / dt

        # -------------------------
        # Publish Odometry
        # -------------------------
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        qx, qy, qz, qw = self.quaternion_from_yaw(self.theta)
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = vth

        self.odom_pub.publish(odom)
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t)
        # -------------------------
        # Publish Joint States
        # -------------------------
        js = JointState()
        js.header.stamp = now.to_msg()
        js.name = ["Revolute 2", "Revolute 3", "Revolute 1", "Revolute 4"]

        left_pos = left_ticks * 2.0 * math.pi / self.ticks_per_rev
        right_pos = right_ticks * 2.0 * math.pi / self.ticks_per_rev

        js.position = [
            left_pos,
            left_pos,
            right_pos,
            right_pos
        ]

        self.joint_pub.publish(js)


def main(args=None):
    rclpy.init(args=args)
    node = DiffDriveNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.ser.close()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()