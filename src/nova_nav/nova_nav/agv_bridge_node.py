#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import serial
import threading
import math
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from rclpy.time import Time

# Optional TF
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


def yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    half = yaw * 0.5
    q.z = math.sin(half)
    q.w = math.cos(half)
    q.x = 0.0
    q.y = 0.0
    return q


class AgvBridgeNode(Node):
    def __init__(self):
        super().__init__('agv_bridge_node')

        # Parameters
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)

        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.publish_tf = self.get_parameter('publish_tf').get_parameter_value().bool_value

        # Serial connection
        try:
            self.ser = serial.Serial(
                port=port,
                baudrate=baud,
                timeout=0.05,    # non-blocking-ish
            )
            self.get_logger().info(f'Opened serial port {port} @ {baud}')
        except Exception as e:
            self.get_logger().error(f'Failed to open serial port {port}: {e}')
            raise

        # ROS interfaces
        self.cmd_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.odom_pub = self.create_publisher(
            Odometry,
            'odom',
            10
        )

        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)
        else:
            self.tf_broadcaster = None

        # State from last O: line
        self.last_x = 0.0
        self.last_y = 0.0
        self.last_theta = 0.0
        self.last_v = 0.0
        self.last_w = 0.0

        # Thread to read from serial
        self._stop_event = threading.Event()
        self.read_thread = threading.Thread(target=self.serial_reader_loop, daemon=True)
        self.read_thread.start()

        # Timer to republish odom at a fixed rate (e.g. 20 Hz)
        self.odom_timer = self.create_timer(0.05, self.publish_odom)

    # ---------------- CMD_VEL -> Serial -----------------
    def cmd_vel_callback(self, msg: Twist):
        v = msg.linear.x
        w = msg.angular.z
        # Format: V:<v> W:<w>\n
        line = f"V:{v:.3f} W:{w:.3f}\n"
        try:
            self.ser.write(line.encode('ascii'))
        except Exception as e:
            self.get_logger().warn(f'Failed to write to serial: {e}')

    # ---------------- Serial -> ODOM --------------------
    def serial_reader_loop(self):
        """
        Read lines like:
          O:x y theta v w
        from the Mega and store them in variables.
        """
        buffer = b''
        while not self._stop_event.is_set():
            try:
                chunk = self.ser.read(64)
                if chunk:
                    buffer += chunk
                    # Split into lines
                    while b'\n' in buffer:
                        line, buffer = buffer.split(b'\n', 1)
                        line_str = line.decode('ascii', errors='ignore').strip()
                        self.handle_serial_line(line_str)
                else:
                    # no data, sleep a bit
                    time.sleep(0.005)
            except Exception as e:
                self.get_logger().warn(f'Serial read error: {e}')
                time.sleep(0.1)

    def handle_serial_line(self, line: str):
        # Expect lines like: O:0.523 0.012 0.105 0.200 0.000
        if not line:
            return
        if not line.startswith('O:'):
            # You can print debug for unknown lines if needed
            # self.get_logger().debug(f'Unknown serial line: {line}')
            return

        try:
            payload = line[2:].strip()
            parts = payload.split()
            if len(parts) < 5:
                self.get_logger().warn(f'Bad O line (not enough parts): {line}')
                return

            x = float(parts[0])
            y = float(parts[1])
            theta = float(parts[2])
            v = float(parts[3])
            w = float(parts[4])

            self.last_x = x
            self.last_y = y
            self.last_theta = theta
            self.last_v = v
            self.last_w = w

        except ValueError:
            self.get_logger().warn(f'Failed to parse O line: {line}')

    # ---------------- Publish /odom (+ TF) ---------------
    def publish_odom(self):
        now = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.last_x
        odom.pose.pose.position.y = self.last_y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = yaw_to_quat(self.last_theta)

        odom.twist.twist.linear.x = self.last_v
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = self.last_w

        self.odom_pub.publish(odom)

        # Optional TF
        if self.tf_broadcaster is not None:
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame

            t.transform.translation.x = self.last_x
            t.transform.translation.y = self.last_y
            t.transform.translation.z = 0.0

            q = yaw_to_quat(self.last_theta)
            t.transform.rotation = q

            self.tf_broadcaster.sendTransform(t)

    # ---------------- Shutdown --------------------------
    def destroy_node(self):
        self._stop_event.set()
        try:
            self.read_thread.join(timeout=1.0)
        except RuntimeError:
            pass
        try:
            self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AgvBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
