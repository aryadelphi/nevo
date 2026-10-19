#!/usr/bin/env python3
"""
IMU publisher for Arduino Nano + MPU6050.
Reads CSV: ax,ay,az (g), gx,gy,gz (deg/s), temp (C)*HH; lines starting with '#' are comments.
HH is the XOR of all characters before '*' (NMEA style). Lines with a bad checksum or
values outside the sensor's configured range (+/-250 deg/s, +/-2 g) are dropped: the
USB-serial link occasionally garbles a line (seen as gz up to 36 rad/s, 2026-10-18).
Publishes sensor_msgs/Imu on /imu/data_raw (m/s^2, rad/s), one message per sample.

Tilt correction: the board is not mounted flat, so readings are rotated into the
robot frame using the robot's "up" axis expressed in sensor coordinates.
  tilt_correction = 'fixed'   -> use the mount_up parameter (measured once)
  tilt_correction = 'gravity' -> average the accelerometer at startup (robot must be
                                 still and on level floor)
  tilt_correction = 'none'    -> publish raw sensor axes
Only roll/pitch are corrected; the mount yaw (sensor x vs robot x) is not observable
from gravity and does not affect angular_velocity.z.
"""
import math
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import serial

G = 9.80665
DEG2RAD = math.pi / 180.0
ACC_LIMIT_G = 2.05     # +/-2 g full scale, small margin for saturated readings
GYRO_LIMIT_DPS = 252.0  # +/-250 deg/s full scale


def parse_line(line, require_checksum=True):
    """Return (ax, ay, az, gx, gy, gz) in g and deg/s, or None for comments/garbage."""
    if not line or line.startswith('#'):
        return None
    if '*' in line:
        line, cs = line.rsplit('*', 1)
        x = 0
        for ch in line:
            x ^= ord(ch)
        try:
            if int(cs, 16) != x:
                return None
        except ValueError:
            return None
    elif require_checksum:
        return None
    parts = line.split(',')
    if len(parts) != 7:
        return None
    try:
        s = tuple(float(p) for p in parts[:6])
    except ValueError:
        return None
    if max(abs(v) for v in s[:3]) > ACC_LIMIT_G or max(abs(v) for v in s[3:]) > GYRO_LIMIT_DPS:
        return None
    return s


def rotation_to_up(up):
    """3x3 rotation R (row-major lists) with R @ up_unit = [0, 0, 1]."""
    n = math.sqrt(sum(c * c for c in up))
    ux, uy, uz = (c / n for c in up)
    # v = u x z, c = u . z  (Rodrigues for aligning two unit vectors)
    vx, vy, vz = uy, -ux, 0.0
    c = uz
    if c < -0.999999:
        raise ValueError('up vector points down; mount is upside down')
    k = 1.0 / (1.0 + c)
    return [
        [1 - k * (vy * vy + vz * vz), k * vx * vy - vz, k * vx * vz + vy],
        [k * vx * vy + vz, 1 - k * (vx * vx + vz * vz), k * vy * vz - vx],
        [k * vx * vz - vy, k * vy * vz + vx, 1 - k * (vx * vx + vy * vy)],
    ]


def rotate(R, v):
    return tuple(R[i][0] * v[0] + R[i][1] * v[1] + R[i][2] * v[2] for i in range(3))


class IMUPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')
        self.declare_parameter('serial_port', '/dev/ttyIMU')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('reset_on_connect', False)  # DTR pulse reboots the Nano -> gyro recalibration
        self.declare_parameter('tilt_correction', 'fixed')  # fixed | gravity | none
        # robot +z in sensor axes: rotation axis of the gyro during in-place spins, 2026-10-18
        # (4.5 deg). The accelerometer said 8.7 deg but it has an offset (reads 1.12 g).
        self.declare_parameter('mount_up', [0.0784, 0.0014, 0.9969])
        self.declare_parameter('gravity_samples', 200)  # 2 s at 100 Hz
        # sensor gz scale vs lidar ICP over 6 spins at 0.2/0.35/0.5 rad/s, 2026-10-18
        self.declare_parameter('gyro_scale_z', 1.0131)
        self.declare_parameter('require_checksum', True)
        self.declare_parameter('gyro_cov', [1e-4, 1e-4, 1e-5])  # rad^2/s^2, x y z
        self.declare_parameter('accel_cov', [0.05, 0.05, 0.05])  # m^2/s^4

        self.port = self.get_parameter('serial_port').value
        self.baud = self.get_parameter('baud_rate').value
        self.frame_id = self.get_parameter('frame_id').value
        self.reset_on_connect = self.get_parameter('reset_on_connect').value
        self.tilt_mode = self.get_parameter('tilt_correction').value
        self.gravity_samples = self.get_parameter('gravity_samples').value
        self.gyro_scale_z = self.get_parameter('gyro_scale_z').value
        self.require_checksum = self.get_parameter('require_checksum').value
        gc = self.get_parameter('gyro_cov').value
        ac = self.get_parameter('accel_cov').value
        self.gyro_cov = [gc[0], 0.0, 0.0, 0.0, gc[1], 0.0, 0.0, 0.0, gc[2]]
        self.accel_cov = [ac[0], 0.0, 0.0, 0.0, ac[1], 0.0, 0.0, 0.0, ac[2]]

        self.R = None
        self.gravity_acc = []
        if self.tilt_mode == 'fixed':
            self.set_up_vector(list(self.get_parameter('mount_up').value))
        elif self.tilt_mode == 'none':
            self.R = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        elif self.tilt_mode != 'gravity':
            raise ValueError(f"tilt_correction must be fixed|gravity|none, got '{self.tilt_mode}'")

        self.pub = self.create_publisher(Imu, '/imu/data_raw', 10)
        self.ser = None
        self.msg_count = 0
        self.lines_ok = 0
        self.lines_bad = 0
        self.last_line = None
        self.last_ok = time.monotonic()
        self.running = True
        self.thread = threading.Thread(target=self.read_loop, daemon=True)
        self.thread.start()
        self.create_timer(10.0, self.report_link)
        self.create_timer(1.0, self.watchdog)
        self.get_logger().info(
            f'IMU publisher on {self.port} @ {self.baud} baud, tilt_correction={self.tilt_mode}, '
            f'gyro_scale_z={self.gyro_scale_z}, require_checksum={self.require_checksum}')

    def report_link(self):
        ok, bad = self.lines_ok, self.lines_bad
        self.lines_ok = self.lines_bad = 0
        if bad:
            self.get_logger().warn(f'IMU link: rejected {bad} of {ok + bad} lines in the last 10 s')
        elif ok == 0:
            self.get_logger().warn('IMU link: no valid lines in the last 10 s')

    def watchdog(self):
        # 2026-10-19: the MPU died mid-run and the node went silent with only a periodic
        # 'rejected' warning. The EKF quietly falls back to wheels + rf2o, so say it loudly.
        gap = time.monotonic() - self.last_ok
        if gap > 1.0:
            self.get_logger().error(
                f'IMU: no valid sample for {gap:.0f} s, EKF running without IMU '
                f'(check MPU6050 wiring / replug the Nano)', throttle_duration_sec=5.0)

    def set_up_vector(self, up):
        self.R = rotation_to_up(up)
        n = math.sqrt(sum(c * c for c in up))
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, up[2] / n))))
        self.get_logger().info(
            f'Tilt correction: up=[{up[0]/n:.4f}, {up[1]/n:.4f}, {up[2]/n:.4f}] '
            f'({tilt:.2f} deg from sensor z)')

    def connect_serial(self):
        try:
            # exclusive: a second reader on the same tty splits the byte stream and shows up
            # as endless 'disconnected' errors; fail loudly instead
            self.ser = serial.Serial(self.port, self.baud, timeout=0.5, exclusive=True)
            if self.reset_on_connect:
                self.ser.dtr = False
                time.sleep(0.1)
                self.ser.dtr = True
                time.sleep(3.0)  # bootloader + gyro calibration; keep the robot still
            self.ser.reset_input_buffer()
            self.get_logger().info(f'Serial connected: {self.port}')
        except serial.SerialException as e:
            self.get_logger().error(f'Serial connection failed: {e}', throttle_duration_sec=10.0)
            self.ser = None

    def read_loop(self):
        while self.running and rclpy.ok():
            if self.ser is None or not self.ser.is_open:
                self.connect_serial()
                if self.ser is None:
                    threading.Event().wait(1.0)
                    continue
            try:
                raw = self.ser.readline()
            except serial.SerialException:
                self.get_logger().error('Serial disconnected, reconnecting...', throttle_duration_sec=5.0)
                self.ser = None
                continue
            line = raw.decode('utf-8', errors='ignore').strip()
            if not line:
                continue
            # A stuck CH340 replays its last USB packet thousands of times per second
            # (seen after flashing and on port open). Real samples never repeat exactly
            # (noise + temperature), so an identical line is stale, even with a valid checksum.
            # Checked before comments so a replayed '#' line is not logged hundreds of times.
            if line == self.last_line:
                self.lines_bad += 1
                continue
            self.last_line = line
            if line.startswith('#'):
                self.get_logger().info(f'IMU: {line}')
                continue
            sample = parse_line(line, self.require_checksum)
            if sample is None:
                self.lines_bad += 1
                continue
            self.lines_ok += 1
            self.last_ok = time.monotonic()
            self.handle_sample(sample)

    def handle_sample(self, s):
        if self.R is None:  # gravity mode, still collecting
            self.gravity_acc.append(s[:3])
            if len(self.gravity_acc) >= self.gravity_samples:
                n = len(self.gravity_acc)
                self.set_up_vector([sum(a[i] for a in self.gravity_acc) / n for i in range(3)])
                self.gravity_acc = []
            return

        acc = rotate(self.R, (s[0] * G, s[1] * G, s[2] * G))
        gyr = rotate(self.R, (s[3] * DEG2RAD, s[4] * DEG2RAD, s[5] * DEG2RAD * self.gyro_scale_z))

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        # Orientation not measured
        msg.orientation_covariance[0] = -1.0
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = gyr
        msg.angular_velocity_covariance = self.gyro_cov
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = acc
        msg.linear_acceleration_covariance = self.accel_cov
        self.pub.publish(msg)

        self.msg_count += 1
        if self.msg_count == 1:
            self.get_logger().info(f'First IMU message: az={acc[2]:.2f} gz={gyr[2]:.4f}')

    def destroy_node(self):
        self.running = False
        if self.ser is not None:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = IMUPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
