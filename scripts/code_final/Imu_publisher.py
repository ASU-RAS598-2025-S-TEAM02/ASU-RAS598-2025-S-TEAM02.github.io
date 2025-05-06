#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
from scipy.signal import butter, lfilter
import numpy as np

class ESP32IMUNode(Node):
    def __init__(self):
        super().__init__('esp32_imu_node')

        self.serial_port = '/dev/ttyUSB0'
        self.baudrate = 115200

        try:
            self.ser = serial.Serial(self.serial_port, self.baudrate, timeout=1)
            self.get_logger().info(f"Serial connected on {self.serial_port} at {self.baudrate} baud")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial connection failed: {e}")
            raise SystemExit

        self.pub_imu = self.create_publisher(Imu, '/esp_05/imu_data', 10)
        self.timer = self.create_timer(0.01, self.timer_callback)  # 100 Hz

        # Butterworth filter setup
        self.order = 2
        self.cutoff = 2.0  # Hz
        self.sampling_rate = 100.0  # Hz
        self.b, self.a = butter(self.order, self.cutoff / (0.5 * self.sampling_rate), btype='low')
        self.buffer_size = 10
        self.ax_buf, self.ay_buf, self.az_buf = [], [], []

    def timer_callback(self):
        if self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode('utf-8').strip()
                imu_msg = self.parse_imu_line(line)
                if imu_msg:
                    self.pub_imu.publish(imu_msg)
            except Exception as e:
                self.get_logger().warn(f"Serial read/parse error: {e}")

    def filter_signal(self, buffer, new_val):
        buffer.append(new_val)
        if len(buffer) > self.buffer_size:
            buffer.pop(0)
        if len(buffer) < self.buffer_size:
            return new_val
        return lfilter(self.b, self.a, buffer)[-1]

    def parse_imu_line(self, line):
        fields = line.split(',')
        if len(fields) != 10:
            return None
        try:
            ax, ay, az, gx, gy, gz, qx, qy, qz, qw = map(float, fields)

            # Filter acceleration
            ax_f = self.filter_signal(self.ax_buf, ax)
            ay_f = self.filter_signal(self.ay_buf, ay)
            az_f = self.filter_signal(self.az_buf, az)

            # Apply ZUPT: Zero acceleration (no motion)
            if np.linalg.norm([ax_f, ay_f, az_f]) < 0.05:
                ax_f, ay_f, az_f = 0.0, 0.0, 0.0

            imu_msg = Imu()
            imu_msg.header = Header()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = 'base_link'

            imu_msg.linear_acceleration.x = ax_f
            imu_msg.linear_acceleration.y = ay_f
            imu_msg.linear_acceleration.z = az_f

            imu_msg.angular_velocity.x = gx
            imu_msg.angular_velocity.y = gy
            imu_msg.angular_velocity.z = gz

            imu_msg.orientation.x = qx
            imu_msg.orientation.y = qy
            imu_msg.orientation.z = qz
            imu_msg.orientation.w = qw

            imu_msg.orientation_covariance = [0.01, 0.0, 0.0,
                                              0.0, 0.01, 0.0,
                                              0.0, 0.0, 0.01]
            imu_msg.angular_velocity_covariance = [0.01]*9
            imu_msg.linear_acceleration_covariance = [0.01]*9

            return imu_msg
        except ValueError:
            return None

    def destroy_node(self):
        if self.ser.is_open:
            self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ESP32IMUNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

