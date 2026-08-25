#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
import tf2_ros
import math, time, csv, os
from datetime import datetime

# Import your custom telemetry message
from nevo_msgs.msg import NevoTelemetry

def camera_yaw_from_quaternion(x, y, z, w):
    """ Extracts true steering angle (Pitch Y-Axis) from an optical frame. """
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1:
        yaw_rad = math.copysign(math.pi / 2, sinp)
    else:
        yaw_rad = math.asin(sinp)
    return math.degrees(yaw_rad)

def map_yaw_from_quaternion(x, y, z, w):
    """ Extracts standard Yaw (Z-Axis) from a map/odom frame. """
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    return math.degrees(math.atan2(siny_cosp, cosy_cosp))

class PureDock(Node):
    def __init__(self):
        super().__init__('pure_dock')
        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        self.docked = False
            
        def telemetry_cb(msg):
            # We only care about the physical limit switch now, not the FSM state
            if msg.is_docked and not self.docked:
                self.get_logger().info('*** LIMIT SWITCH TRIGGERED ***')
            self.docked = msg.is_docked
                
        self.create_subscription(NevoTelemetry, '/nevo_telemetry', telemetry_cb, 10)
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # CSV Setup (New filename to protect your full-lifecycle thesis data)
        self.csv_filename = os.path.expanduser('~/pure_docking_cycles_log.csv')
        self.setup_csv()

    def setup_csv(self):
        file_exists = os.path.isfile(self.csv_filename)
        with open(self.csv_filename, 'a', newline='') as f:
            writer = csv.writer(f)
            if not file_exists:
                writer.writerow([
                    'Timestamp', 'Cycle_Time_s',  
                    'Start_X', 'Start_Y', 'Start_Yaw_deg',
                    'Staging_X', 'Staging_Y', 'Staging_Yaw_deg',
                    'Final_Error_X_cm', 'Final_Error_Yaw_deg', 'Result'
                ])

    def get_tag(self):
        try:
            t = self.tf_buffer.lookup_transform(
                'c310_camera_link_optical', 'tag36h11:5', rclpy.time.Time())
            
            tf_time = t.header.stamp.sec + (t.header.stamp.nanosec / 1e9)
            now = self.get_clock().now().nanoseconds / 1e9
            
            if now - tf_time > 0.5:
                return None, None, None
                
            trans = t.transform.translation
            rot = t.transform.rotation
            yaw = camera_yaw_from_quaternion(rot.x, rot.y, rot.z, rot.w)
            
            return trans, rot, yaw
        except Exception:
            return None, None, None

    def get_current_map_pose(self):
        """ Instantly queries the TF tree for the robot's location on the map """
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            x = t.transform.translation.x
            y = t.transform.translation.y
            rot = t.transform.rotation
            yaw = map_yaw_from_quaternion(rot.x, rot.y, rot.z, rot.w)
            return x, y, yaw
        except Exception:
            return None, None, None

    def wait_for_nav2(self):
        for i in range(120):
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.nav_client.server_is_ready():
                self.get_logger().info(f'Nav2 server found after {i+1}s')
                return True
            time.sleep(1.0)
        self.get_logger().error('Nav2 action server not found!')
        return False

    def navigate_to(self, x, y, yaw_rad, label='goal'):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw_rad / 2)
        goal.pose.pose.orientation.w = math.cos(yaw_rad / 2)

        nav_done = [False]
        nav_ok = [False]

        def cb(future):
            gh = future.result()
            if not gh.accepted:
                self.get_logger().error(f'{label} goal rejected!')
                nav_done[0] = True
                return
            self.get_logger().info(f'Navigating to {label}...')
            gh.get_result_async().add_done_callback(lambda f: (
                nav_ok.__setitem__(0, f.result().status == 4),
                nav_done.__setitem__(0, True)
            ))

        self.nav_client.send_goal_async(goal).add_done_callback(cb)
        while rclpy.ok() and not nav_done[0]:
            rclpy.spin_once(self, timeout_sec=0.1)
        return nav_ok[0]

    def run(self):
        # ==========================================
        # 📍 STAGING COORDINATES
        # ==========================================
        STAGING_X = -0.00630409
        STAGING_Y = -0.192918
        STAGING_YAW_RAD = -0.250428
        # ==========================================

        if not self.wait_for_nav2():
            return

        self.get_logger().warn('🚀 INITIATING PURE DOCKING SEQUENCE!')
        
        # ⏱ START MASTER TIMER
        master_start_time = time.time()
        timestamp_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        # === Phase 1.1: Record Start Location via TF ===
        self.get_logger().info('⏳ Fetching start pose from TF tree...')
        start_x, start_y, start_yaw = None, None, None
        while rclpy.ok() and start_x is None:
            start_x, start_y, start_yaw = self.get_current_map_pose()
            if start_x is None:
                rclpy.spin_once(self, timeout_sec=0.1)
                
        self.get_logger().info(f'📍 Start Pose -> X: {start_x:.3f}, Y: {start_y:.3f}, Yaw: {start_yaw:.1f}°')

        # ==========================================
        # 🔄 INFINITE RETRY DOCKING LOOP
        # ==========================================
        docking_success = False
        while rclpy.ok() and not docking_success:

            # === Phase 1.5: Navigate to Staging ===
            self.get_logger().info('--- Phase 1.5: Navigating to Staging Area ---')
            if not self.navigate_to(STAGING_X, STAGING_Y, STAGING_YAW_RAD, 'staging'):
                self.get_logger().error('Failed to reach staging area. Retrying...')
                time.sleep(2.0)
                continue  # Jump back to top of retry loop

            # === Phase 1.6: Record Staging Location ===
            time.sleep(1.0) # Let robot settle
            stage_x, stage_y, stage_yaw = self.get_current_map_pose()
            self.get_logger().info(f'📍 Staging Pose -> X: {stage_x:.3f}, Y: {stage_y:.3f}, Yaw: {stage_yaw:.1f}°')

            # === Phase 2: Radar sweep ===
            self.get_logger().info('--- Phase 2: Radar sweep ---')
            sweep_start = time.time()
            while rclpy.ok() and time.time() - sweep_start < 45.0 and not self.docked:
                trans, rot, yaw = self.get_tag()
                if trans is not None:
                    self.get_logger().info(f'Tag found! Z={abs(trans.z):.3f}m, X={trans.x:.3f}m')
                    break
                cmd = Twist()
                cmd.angular.z = 0.15
                self.cmd_pub.publish(cmd)
                rclpy.spin_once(self, timeout_sec=0.1)
            self.cmd_pub.publish(Twist())

            # === Phase 3: Cascade Creep until 40cm Gate ===
            self.get_logger().info('--- Phase 3: Creep to Dock Entrance ---')
            k_x = 0.2
            k_yaw = 0.02
            approach_speed = -0.05
            
            creep_start = time.time()
            terminal_push_ready = False
            last_seen_time = time.time()
            last_tag_x = 0.0  

            while rclpy.ok() and time.time() - creep_start < 180.0 and not self.docked:
                trans, rot, yaw = self.get_tag()
                cmd = Twist()
                
                if trans is not None:
                    last_seen_time = time.time()
                    tag_z = abs(trans.z)
                    tag_x = trans.x
                    last_tag_x = tag_x  
                    
                    raw_steering = (tag_x * k_x) - (yaw * k_yaw)
                    max_steer = 0.05
                    cmd.linear.x = approach_speed
                    cmd.angular.z = max(-max_steer, min(max_steer, raw_steering))
                else:
                    lost_duration = time.time() - last_seen_time
                    if lost_duration > 8.0:
                        self.get_logger().warn('⚠ Tag completely lost. Aborting approach!')
                        break
                    
                    cmd.linear.x = 0.0   
                    search_spin_speed = 0.15
                    cmd.angular.z = -search_spin_speed if last_tag_x > 0 else search_spin_speed
                    
                    direction = "RIGHT" if last_tag_x > 0 else "LEFT"
                    self.get_logger().warn(f'Tag lost! Searching {direction}...', throttle_duration_sec=1.0)
                self.cmd_pub.publish(cmd)
                rclpy.spin_once(self, timeout_sec=0.05)
                
            self.cmd_pub.publish(Twist())
            
            # === Phase 4: The Terminal Push ===
            if terminal_push_ready and not self.docked:
                self.get_logger().info('--- Phase 4: Terminal push into the funnel ---')
                push_start = time.time()
                cmd = Twist()
                cmd.linear.x = approach_speed
                
                while rclpy.ok() and time.time() - push_start < 6.0 and not self.docked:
                    self.cmd_pub.publish(cmd)
                    rclpy.spin_once(self, timeout_sec=0.05)
                self.cmd_pub.publish(Twist())

            # CHECK SUCCESS OR LOOP AGAIN
            if self.docked:
                docking_success = True
            else:
                self.get_logger().error('❌ Docking failed. Restarting from Staging Area...')
                self.log_cycle(timestamp_str, master_start_time, start_x, start_y, start_yaw, stage_x, stage_y, stage_yaw, "N/A", "N/A", "FAILED_RETRYING")
        
        # ==========================================
        # 🎯 RECORD PRECISION ERRORS & FINISH
        # ==========================================
        self.get_logger().info('Fetching final TF errors...')
        time.sleep(0.5)   
        final_x_cm = "N/A"
        final_yaw_deg = "N/A"
        try:
            t = self.tf_buffer.lookup_transform('c310_camera_link_optical', 'tag36h11:5', rclpy.time.Time())
            final_x_cm = t.transform.translation.x * 100.0
            rot = t.transform.rotation
            final_yaw_deg = camera_yaw_from_quaternion(rot.x, rot.y, rot.z, rot.w)
            self.get_logger().info(f"🎯 Final Precision -> X Error: {final_x_cm:.1f}cm | Yaw Error: {final_yaw_deg:.2f}°")
        except Exception as e:
            self.get_logger().warn(f"Docked, but failed to get final TF for logging: {e}")

        # ⏱ STOP MASTER TIMER & LOG
        self.log_cycle(timestamp_str, master_start_time, start_x, start_y, start_yaw, stage_x, stage_y, stage_yaw, final_x_cm, final_yaw_deg, "SUCCESS")
        self.get_logger().info('🎉 PURE DOCKING COMPLETE & LOGGED!')

    def log_cycle(self, t_stamp, start_time, sx, sy, syaw, stgx, stgy, stgyaw, fx, fyaw, result):
        total_time = time.time() - start_time
        
        def fmt(val):
            return f"{val:.3f}" if isinstance(val, float) else val
            
        with open(self.csv_filename, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                t_stamp, f"{total_time:.1f}",  
                fmt(sx), fmt(sy), fmt(syaw),
                fmt(stgx), fmt(stgy), fmt(stgyaw),
                fmt(fx), fmt(fyaw), result
            ])
        self.get_logger().info(f"💾 Cycle data saved to {self.csv_filename}")

def main():
    rclpy.init()
    node = PureDock()
    node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
