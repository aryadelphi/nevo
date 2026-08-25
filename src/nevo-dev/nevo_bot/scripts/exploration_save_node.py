#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray
from slam_toolbox.srv import SerializePoseGraph
import subprocess
import os
import time

class ExplorationSaveNode(Node):
    def __init__(self):
        super().__init__('exploration_save_node')
        
        # 1. Subscribe to frontiers
        self.subscription = self.create_subscription(
            MarkerArray,
            '/explore/frontiers',
            self.frontier_callback,
            10)
        
        # 2. Client to save SLAM state
        self.cli_serialize = self.create_client(SerializePoseGraph, '/slam_toolbox/serialize_map')
        
        # 3. BACKGROUND TIMER
        # Check the status every 1.0 second
        self.timer = self.create_timer(1.0, self.check_status_callback)
        
        # State variables
        self.exploration_started = False
        self.frontier_count = -1 
        self.last_msg_time = None  # NEW: Track when we last heard from explorer
        self.no_frontiers_start_time = None
        self.save_triggered = False
        
        # SETTINGS
        self.WAIT_FOR_RETURN_DURATION = 5.0 # Reduced to 5s for testing
        self.SILENCE_TIMEOUT = 5.0          # If silent for 5s, assume finished
        
        self.home_dir = os.path.expanduser('~')
        self.map_name = "auto_saved_map"

        self.get_logger().info('--- AUTO-SAVE MANAGER V3 STARTED ---')
        self.get_logger().info(f'Will save {self.WAIT_FOR_RETURN_DURATION}s after exploration ends.')

    def frontier_callback(self, msg):
        self.frontier_count = len(msg.markers)
        self.last_msg_time = time.time() # Update timestamp whenever we hear something
        
        # Detect start of exploration
        if self.frontier_count > 0:
            if not self.exploration_started:
                self.get_logger().info('Exploration detected! Monitoring...')
            self.exploration_started = True
            self.no_frontiers_start_time = None 

    def check_status_callback(self):
        if self.save_triggered or not self.exploration_started:
            return

        # CHECK 1: Explicitly finished (count is 0)
        is_zero = (self.frontier_count == 0)
        
        # CHECK 2: Implicitly finished (Topic is silent)
        is_silent = False
        if self.last_msg_time is not None:
            time_since_msg = time.time() - self.last_msg_time
            if time_since_msg > self.SILENCE_TIMEOUT:
                is_silent = True
        
        # If either condition is met, start the countdown
        if is_zero or is_silent:
            if self.no_frontiers_start_time is None:
                self.no_frontiers_start_time = time.time()
                reason = "Zero frontiers" if is_zero else "Topic silence"
                self.get_logger().info(f'Exploration finished ({reason}). Countdown started...')
            
            # Calculate countdown
            elapsed = time.time() - self.no_frontiers_start_time
            remaining = self.WAIT_FOR_RETURN_DURATION - elapsed

            # Log countdown occasionally
            if elapsed < self.WAIT_FOR_RETURN_DURATION:
                self.get_logger().info(f'Saving map in {int(remaining)} seconds...', throttle_duration_sec=1.0)

            # Trigger Save
            if elapsed > self.WAIT_FOR_RETURN_DURATION:
                self.trigger_save_sequence()

    def trigger_save_sequence(self):
        self.save_triggered = True
        self.get_logger().info('TIMEOUT REACHED - SAVING MAP DATA NOW...')
        
        # 1. Save Map Image
        map_path = os.path.join(self.home_dir, self.map_name)
        self.get_logger().info(f'[1/2] Saving Map Image to {map_path}...')
        try:
            subprocess.run(['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', map_path], check=True)
            self.get_logger().info('Map Image Saved Successfully!')
        except Exception as e:
            self.get_logger().error(f'Failed to save Map Image: {e}')

        # 2. Serialize SLAM Toolbox
        serial_path = os.path.join(self.home_dir, self.map_name)
        self.get_logger().info(f'[2/2] Serializing SLAM Pose Graph to {serial_path}...')
        
        if self.cli_serialize.wait_for_service(timeout_sec=2.0):
            req = SerializePoseGraph.Request()
            req.filename = serial_path
            self.cli_serialize.call_async(req)
            self.get_logger().info('Serialization Request Sent!')
        else:
            self.get_logger().warn('SLAM Toolbox service not available. Skipping serialization.')

        self.get_logger().info('--- ALL DONE. YOU CAN CLOSE TERMINAL ---')

def main(args=None):
    rclpy.init(args=args)
    node = ExplorationSaveNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()