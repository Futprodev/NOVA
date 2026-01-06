import math
import time
import threading
import re
from typing import Optional, Tuple, List

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String, Bool, Empty
from sensor_msgs.msg import JointState

import serial

def deg2rad(degrees: float) -> float:
    return math.radians(degrees)

class ArmBridgeNode(Node):
    def __init__(self):
        super().__init__('nova_arm_bridge')

        # serial setting
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)

        # Joint name
        self.declare_parameter('joint_names', [
            "base_joint", "shoulder_joint", "elbow_joint",
        ])

        # sequence done
        self.declare_parameter('seq_done_mode', 'pulse')  # or 'latched'
        self.declare_parameter('seq_done_pulse_s', 0.2)

        self.declare_parameter('query_rate_hz', 5.0)
        self.declare_parameter('publish_rate_hz', 20.0)

        port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value

        self.joint_names : List[str] = [
            s for s in self.get_parameter('joint_names').get_parameter_value().string_array_value
        ]

        self.seq_done_mode = self.get_parameter('seq_done_mode').get_parameter_value().string_value.strip().lower()
        self.seq_done_pulse_s = float(self.get_parameter('seq_done_pulse_s').value)

        self.query_rate_hz = self.get_parameter('query_rate_hz').get_parameter_value().double_value
        self.publish_rate_hz = self.get_parameter('publish_rate_hz').get_parameter_value().double_value

        if len(self.joint_names) != 3:
            self.get_logger().warn("joint_names should have 3 names; falling back to defaults.")
            self.joint_names = ['joint_base', 'joint_shoulder', 'joint_elbow']

        try:
            self.serial_com = serial.Serial(
                port=port,
                baudrate=baud_rate,
                timeout=0.05,
                write_timeout=0.1
            )

            self.get_logger().info(f'Opened serial port {port} @ {baud_rate}')
        except Exception as e:
            self.get_logger().error(f'Failed to open serial port {port}: {e}')
            raise

        # pub sub
        self.joint_pub = self.create_publisher(JointState, '/arm_joint_states', 10)
        self.joint_deg_pub = self.create_publisher(Float64MultiArray, '/arm_joint_deg', 10)
        self.status_pub = self.create_publisher(String, 'nova_arm/status', 10)
        self.seq_done_pub = self.create_publisher(Bool, 'nova_arm/seq_done', 10)

        self.target_sub = self.create_subscription(
            Float64MultiArray,
            "nova_arm/command_deg",
            self.on_target_deg,
            10
        )

        self.grip_sub = self.create_subscription(
            Bool,
            'nova_arm/grip_command',
            self.on_gripper,
            10
        )

        self.home_sub = self.create_subscription(
            Empty,
            'nova_arm/home_command',
            self.on_home,
            10
        )

        self.reset_sub = self.create_subscription(
            Empty,
            'nova_arm/reset',
            self.on_reset,
            10
        )

        # states
        self._lock = threading.Lock()
        self.last_deg: Optional[Tuple[float, float, float]] = None
        self.last_rx_time = self.get_clock().now()

            # -- Regex patterns
        self._re_bs = re.compile(r'BS=([-\d.]+|NaN)', re.IGNORECASE)
        self._re_sh = re.compile(r'SH=([-\d.]+|NaN)', re.IGNORECASE)
        self._re_el = re.compile(r'EL=([-\d.]+|NaN)', re.IGNORECASE)

        # start threads
        self._stop_event = threading.Event()
        self.read_thread = threading.Thread(target=self._serial_reader_loop, daemon=True)
        self.read_thread.start()

        # seq state / timer
        self._seq_done_state = False
        self._seq_done_clear_timer = None

        if self.query_rate_hz > 0.0:
            self.query_timer = self.create_timer(1.0 / self.query_rate_hz, self._send_query)

        self.pub_timer = self.create_timer(1.0 / self.publish_rate_hz, self._publish_joint_states)

        self._publish_seq_done(False)

    def _write_line(self, line: str):
        try:
            self.serial_com.write(line.encode('ascii'))
        except Exception as e:
            self.get_logger().warn(f'Serial write failed: {e}')
    
    def _send_query(self):
        self._write_line("Q\n")

    # seq reset
    def _clear_seq_done_on_new_command(self):
        if self.seq_done_mode == 'latched':
            if self._seq_done_state:
                self._publish_seq_done(False)
        
        else:
            if self._seq_done_clear_timer is not None:
                self._seq_done_clear_timer.cancel()
                self._seq_done_clear_timer = None
            if self._seq_done_state:
                self._publish_seq_done(False)

    # commands
    def on_target_deg(self, msg: Float64MultiArray):
        if len(msg.data) < 3:
            self.get_logger().warn('target_deg needs 3 values: [base, shoulder, elbow]')
            return
    
        b, s, e = float(msg.data[0]), float(msg.data[1]), float(msg.data[2])

        self._clear_seq_done_on_new_command()
        self._write_line(f"T {b:.3f} {s:.3f} {e:.3f}\n")

    def on_gripper(self, msg: Bool):
        st = 1 if msg.data else 0
        self._clear_seq_done_on_new_command()
        self._write_line(f"g {st}\n")

    def on_home(self, _msg: Empty):
        self._clear_seq_done_on_new_command()
        self._write_line("H\n")

    def on_reset(self, _msg: Empty):
        self._clear_seq_done_on_new_command()
        self._write_line("R\n")

    # serial
    def _serial_reader_loop(self):
        buffer = b''

        while not self._stop_event.is_set():
            try:
                chunk = self.serial_com.read(120)
                if chunk:
                    buffer += chunk
                    while b'\n' in buffer:
                        raw, buffer = buffer.split(b'\n', 1)
                        line = raw.decode('ascii', errors='ignore'.strip())
                        if line:
                            self._handle_serial_line(line)

                else:
                    time.sleep(0.005)
            
            except Exception as e:
                self.get_logger().warn(f'Serial read error: {e}')

    def _handle_serial_line(self, line: str):
        if line == "SEQ_DONE":
            self.status_pub.publish(String(data=line))
            self._on_seq_done_event()
            return
        
        if line.startswith("[SEQ]") or line.startswith("[CMD]") or line.startswith("[POSTHOME]") or line.startswith("[GRIP]"):
            self.status_pub.publish(String(data=line))
            return
        
        b = self._extract_float(self._re_bs, line)
        s = self._extract_float(self._re_sh, line)
        e = self._extract_float(self._re_el, line)

        if b is not None and s is not None and e is not None:
            with self._lock:
                self.last_deg = (b, s, e)
            return
        
    def _on_seq_done_event(self):
        self._publish_seq_done(True)

        if self.seq_done_mode == 'pulse':
            if self._seq_done_clear_timer is not None:
                self._seq_done_clear_timer.cancel()
                self._seq_done_clear_timer = None

            def clear_once():
                self._publish_seq_done(False)
                if self._seq_done_clear_timer is not None:
                    self._seq_done_clear_timer.cancel()
                    self._seq_done_clear_timer = None

            self._seq_done_clear_timer = self.create_timer(self.seq_done_pulse_s, clear_once)

    # seq and joints
    def _publish_seq_done(self, value: bool):
        self._seq_done_state = value
        self.seq_done_pub.publish(Bool(data=value))

    def _publish_joint_states(self):
        with self._lock:
            if self.last_deg is None:
                return
            b_deg, s_deg, e_deg = self.last_deg

        now = self.get_clock().now().to_msg()

        deg_msg = Float64MultiArray()
        deg_msg.data = [b_deg, s_deg, e_deg]
        self.joint_deg_pub.publish(deg_msg)

        js = JointState()
        js.header.stamp = now
        js.name = self.joint_names
        js.position = [deg2rad(b_deg), deg2rad(s_deg), -1 * deg2rad(e_deg)]
        self.joint_pub.publish(js)

    @staticmethod
    def _extract_float(pattern: re.Pattern, text: str) -> Optional[float]:
        m = pattern.search(text)
        if not m:
            return None
        token = m.group(1)
        if token.lower() == "nan":
            return None
        try:
            return float(token)
        except ValueError:
            return None
    
    def destroy_node(self):
        self._stop_event.set()
        try:
            self.read_thread.join(timeout=1.0)
        except RuntimeError:
            pass
        try:
            self.serial_com.close()
        except Exception:
            pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ArmBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    