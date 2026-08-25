#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math
import time
import threading
import statistics
import argparse
import sys

class LidarValidator(Node):
    def __init__(self):
        super().__init__('lidar_validator')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        self.is_recording = False
        self.target_angle_rad = 0.0
        self.current_angle_deg = 0
        self.samples = []
        self.timestamps = []

    def start_recording(self, angle_deg):
        rad = math.radians(angle_deg)
        while rad > math.pi: rad -= 2 * math.pi
        while rad < -math.pi: rad += 2 * math.pi
        
        self.target_angle_rad = rad
        self.current_angle_deg = angle_deg
        self.samples = []
        self.timestamps = []
        self.is_recording = True

    def stop_recording(self):
        self.is_recording = False

    def scan_callback(self, msg):
        if not self.is_recording:
            return

        if self.target_angle_rad < msg.angle_min or self.target_angle_rad > msg.angle_max:
            return

        index = int((self.target_angle_rad - msg.angle_min) / msg.angle_increment)
        index = max(0, min(index, len(msg.ranges) - 1))

        distance_m = msg.ranges[index]
        timestamp = msg.header.stamp.sec + (msg.header.stamp.nanosec * 1e-9)

        if not math.isinf(distance_m) and not math.isnan(distance_m):
            distance_cm = distance_m * 100.0
            
            self.samples.append(distance_cm)
            self.timestamps.append(timestamp)
            
            # Print live data to terminal exactly once every 10 seconds
            self.get_logger().info(
                f"[TS: {timestamp:.3f}] Angle: {self.current_angle_deg}° | Measured Dist: {distance_cm:.2f} cm",
                throttle_duration_sec=10.0
            )

def main(args=sys.argv):
    ros_args = rclpy.utilities.remove_ros_args(args)
    
    parser = argparse.ArgumentParser(description='LiDAR Single Point Validator (Centimeters)')
    parser.add_argument('-a', '--angle', type=int, required=True,
                        help='Target angle to test in degrees (e.g., -a 90)')
    parser.add_argument('-d', '--distance', type=int, required=True,
                        help='Target distance to test in cm (e.g., -d 100)')
    
    # Changed default recording time to 100 seconds
    parser.add_argument('-t', '--time', type=float, default=100.0,
                        help='Recording duration in seconds (default: 100.0)')
    
    parsed_args = parser.parse_args(ros_args[1:])

    rclpy.init(args=args)
    node = LidarValidator()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    target_angle = parsed_args.angle
    target_distance_cm = parsed_args.distance
    recording_duration = parsed_args.time

    print("=========================================")
    print("  LiDAR Single-Point Validator v1.4 (cm) ")
    print(f" Target Angle:    {target_angle}°")
    print(f" Target Distance: {target_distance_cm} cm")
    print(f" Test Duration:   {recording_duration} seconds")
    print("=========================================")

    try:
        input(f"\n[ACTION] Place the object at {target_angle} degrees, exactly {target_distance_cm} cm away.\nPress [ENTER] when ready to record for {recording_duration} seconds...")
        
        print("\nRecording data... (Outputting to terminal every 10 seconds)")
        node.start_recording(target_angle)
        
        time.sleep(recording_duration)
        
        node.stop_recording()
        print("\n[Recording Complete]")
        
        samples = node.samples
        if len(samples) > 0:
            avg_dist = statistics.mean(samples)
            min_dist = min(samples)
            max_dist = max(samples)
            variance = statistics.variance(samples) if len(samples) > 1 else 0.0
            
            print("\n--- Validation Summary ---")
            print(f"Target Distance : {target_distance_cm:.2f} cm")
            print(f"Total Samples   : {len(samples)}")
            print(f"Average Measured: {avg_dist:.2f} cm")
            print(f"Min / Max       : {min_dist:.2f} cm / {max_dist:.2f} cm")
            print(f"Variance        : {variance:.4f} cm²")
            print("--------------------------")
        else:
            print(f"\nWARNING: No valid data points received for angle {target_angle}°.")
            print("Ensure the object is within range and the LiDAR is publishing.")

        print("\nTest complete. Shutting down.")

    except KeyboardInterrupt:
        print("\nScript interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
