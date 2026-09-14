#!/usr/bin/env python3
"""
IMU publisher for ESP8266 + MPU6050.
Reads CSV: time_ms,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z
Publishes sensor_msgs/Imu on /imu/data_raw.
Drains serial buffer each tick, publishes only the latest reading at the target rate.
"""
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import serial


class IMUPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')
        self.declare_parameter('serial_port', '/dev/ttyIMU')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('publish_rate', 5.0)

        self.port = self.get_parameter('serial_port').value
        self.baud = self.get_parameter('baud_rate').value
        self.frame_id = self.get_parameter('frame_id').value
        rate = self.get_parameter('publish_rate').value

        self.pub = self.create_publisher(Imu, '/imu/data_raw', 10)
        self.ser = None
        self.msg_count = 0
        self.last_data = None

        self.connect_serial()

        # Read serial at 100Hz (fast), publish latest at target rate
        self.read_timer = self.create_timer(0.01, self.read_serial)
        self.pub_timer = self.create_timer(1.0 / rate, self.publish_latest)
        self.get_logger().info(f'IMU publisher started on {self.port} @ {self.baud} baud, publishing at {rate}Hz')

    def connect_serial(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            # DTR/RTS toggle to reset ESP8266 into clean state
            self.ser.dtr = False
            self.ser.rts = True
            time.sleep(0.1)
            self.ser.rts = False
            time.sleep(0.1)
            self.ser.dtr = False
            self.ser.reset_input_buffer()
            time.sleep(2.0)

            # Read until we find the CSV header line
            synced = False
            for _ in range(50):
                raw = self.ser.readline()
                if not raw:
                    continue
                line = raw.decode('utf-8', errors='ignore').strip()
                if line.startswith('time_ms,'):
                    synced = True
                    self.get_logger().info(f'Serial synced on: {line}')
                    break

            if not synced:
                self.get_logger().warn('Could not sync on CSV header, trying anyway')

            self.msg_count = 0
            self.get_logger().info(f'Serial connected: {self.port}')

        except serial.SerialException as e:
            self.get_logger().error(f'Serial connection failed: {e}')
            self.ser = None

    def _parse_line(self, line):
        parts = line.split(',')
        if len(parts) != 7:
            return None
        try:
            return (
                int(parts[0]),
                float(parts[1]), float(parts[2]), float(parts[3]),
                float(parts[4]), float(parts[5]), float(parts[6]),
            )
        except (ValueError, IndexError):
            return None

    def read_serial(self):
        if self.ser is None or not self.ser.is_open:
            self.connect_serial()
            return

        try:
            # Drain all available data, keep only the latest valid reading
            last_valid = None
            while self.ser.in_waiting > 0:
                raw = self.ser.readline()
                if not raw:
                    break
                line = raw.decode('utf-8', errors='ignore').strip()
                if not line:
                    continue
                parsed = self._parse_line(line)
                if parsed is not None:
                    last_valid = parsed

            if last_valid is not None:
                self.last_data = last_valid

        except serial.SerialException:
            self.get_logger().error('Serial disconnected, reconnecting...')
            self.ser = None
        except Exception as e:
            self.get_logger().warn(f'Serial read error: {e}')

    def publish_latest(self):
        if self.last_data is None:
            return

        _time_ms, ax, ay, az, gx, gy, gz = self.last_data

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        # Orientation — not measured (EKF will integrate from gyro)
        msg.orientation.w = 1.0

        # Angular velocity (rad/s) from gyroscope
        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz

        # Gyro covariance — low trust (MPU6050 is noisy)
        msg.angular_velocity_covariance = [
            0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.02
        ]

        # Linear acceleration (m/s²) from accelerometer
        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az

        # Accel covariance — higher (not fused, but set for completeness)
        msg.linear_acceleration_covariance = [
            0.1, 0.0, 0.0,
            0.0, 0.1, 0.0,
            0.0, 0.0, 0.1
        ]

        self.pub.publish(msg)
        self.msg_count += 1
        if self.msg_count == 1:
            self.get_logger().info(f'First IMU message: ax={ax:.2f} gz={gz:.4f}')


def main(args=None):
    rclpy.init(args=args)
    node = IMUPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
