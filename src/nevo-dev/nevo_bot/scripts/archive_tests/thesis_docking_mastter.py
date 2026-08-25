#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import tf2_ros
import time
import csv
import threading
import math

def euler_from_quaternion(x, y, z, w):
    """
    Extracts the true steering angle (Pitch Y-Axis) from an optical frame quaternion.
    Corrects for the camera coordinate frame where Z is forward and Y is down.
    """
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1:
        yaw_rad = math.copysign(math.pi / 2, sinp)
    else:
        yaw_rad = math.asin(sinp)
    return math.degrees(yaw_rad)

class ThesisDockingMaster(Node):
    def __init__(self):
        super().__init__('thesis_docking_master')
        
        # Publishers and Subscribers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Bool, '/dock_state', self.dock_cb, 10)
        
        # TF Listener for Tag Alignment
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.docked = False

        # CSV Setup - Updated for Speed Testing
        self.csv_filename = '/home/robot2/funnel_docking_speed_results.csv'
        self.csv_file = open(self.csv_filename, 'a', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        
        # Write headers if the file is brand new
        self.csv_file.seek(0, 2)
        if self.csv_file.tell() == 0:
            self.csv_writer.writerow([
                'Approach_Speed(m/s)', 'Final_Error_X(cm)', 
                'Final_Yaw(deg)', 'Time_Elapsed(s)', 'Result'
            ])

    def dock_cb(self, msg):
        if msg.data and not self.docked:
            self.get_logger().info('*** LIMIT SWITCH TRIGGERED ***')
        self.docked = msg.data

    def get_tag(self):
        try:
            t = self.tf_buffer.lookup_transform(
                'c310_camera_link_optical', 'tag36h11:5', rclpy.time.Time())
            
            # --- THE GHOST BUSTER ---
            tf_time = t.header.stamp.sec + (t.header.stamp.nanosec / 1e9)
            now = self.get_clock().now().nanoseconds / 1e9
            
            if now - tf_time > 0.5:
                return None, None, None
                
            trans = t.transform.translation
            rot = t.transform.rotation
            yaw = euler_from_quaternion(rot.x, rot.y, rot.z, rot.w)
            
            return trans, rot, yaw
            
        except Exception:
            return None, None, None
            
    def execute_run(self, approach_speed):
        self.docked = False
        start_time = time.time()
        self.get_logger().info(f'--- Starting Test Run at {approach_speed} m/s ---')

        # === Phase 1: Radar Sweep ===
        self.get_logger().info('Phase 1: Radar sweep...')
        angular_speed = 0.15
        sweep_start = time.time()
        
        while rclpy.ok() and time.time() - sweep_start < 45.0 and not self.docked:
            trans, rot, yaw = self.get_tag()
            if trans is not None:
                self.get_logger().info(f'Tag found! Z={abs(trans.z):.3f}m, X={trans.x:.3f}m')
                break
            cmd = Twist()
            cmd.angular.z = angular_speed
            self.cmd_pub.publish(cmd)
            time.sleep(0.1)
        else:
            self.get_logger().info('No tag during sweep, proceeding blindly...')
        self.cmd_pub.publish(Twist())

        # === Phase 2: Creep to Funnel Gate ===
        self.get_logger().info('Phase 2: Creeping to dock entrance (40cm gate)...')
        
        # Cascade Controller Gains
        k_x = 0.1    # Gain for lateral position
        k_yaw = 0.02 # Gain for heading angle
        
        creep_start = time.time()
        terminal_push_ready = False
        abort_recovery = False
        last_seen_time = time.time()  

        while rclpy.ok() and time.time() - creep_start < 180.0 and not self.docked:
            trans, rot, yaw = self.get_tag()
            cmd = Twist()
            
            if trans is not None:
                last_seen_time = time.time()  
                tag_z = abs(trans.z)
                tag_x = trans.x
                
                # Terminal Dashboard Output
                self.get_logger().info(
                    f"Tracking -> Pos(X:{trans.x:.3f}, Y:{trans.y:.3f}, Z:{tag_z:.3f}) | "
                    f"Quat(x:{rot.x:.3f}, y:{rot.y:.3f}, z:{rot.z:.3f}, w:{rot.w:.3f}) | True Yaw: {yaw:.1f}°"
                )
                
                # Check 40cm Gate (Relying entirely on distance now)
                if tag_z <= 0.40:
                    self.get_logger().info(f'✅ 40cm line crossed! Let the physical funnel do the rest. (X={tag_x:.3f}m, Yaw={yaw:.1f}°)')
                    terminal_push_ready = True
                    break
                else:
                    cmd.linear.x = -abs(approach_speed)
                    
                    # The Cascade Controller
                    cmd.angular.z = (tag_x * k_x) - (yaw * k_yaw)
            else:
                if time.time() - last_seen_time > 2.5:
                    self.get_logger().warn('⚠ Tag lost for too long (slippage detected). Aborting run!')
                    abort_recovery = True
                    break
                    
                cmd.linear.x = -0.10

            move_start = time.time()
            while rclpy.ok() and time.time() - move_start < 0.3 and not self.docked:
                self.cmd_pub.publish(cmd)
                time.sleep(0.05)  
            self.cmd_pub.publish(Twist())

            if not self.docked and not terminal_push_ready and not abort_recovery:
                pause_start = time.time()
                while rclpy.ok() and time.time() - pause_start < 0.1 and not self.docked:
                    time.sleep(0.05)  

        # === Phase 3: The Terminal Push ===
        if terminal_push_ready and not self.docked:
            self.get_logger().info(f'Phase 3: Terminal push into the funnel ({-abs(approach_speed)} m/s)...')
            push_start = time.time()
            cmd = Twist()
            cmd.linear.x = -abs(approach_speed)
            cmd.angular.z = 0.0
            
            # Increased timeout to 6.0s to allow for very slow approach speeds
            while rclpy.ok() and time.time() - push_start < 6.0 and not self.docked:
                self.cmd_pub.publish(cmd)
                time.sleep(0.05)  
                
            self.cmd_pub.publish(Twist())
            
            if not self.docked:
                self.get_logger().warn('❌ PUSH FAILED: Timeout reached before limit switch triggered.')
                abort_recovery = True

        elapsed_time = time.time() - start_time

        # === Phase 4: Abort & Return to Staging ===
        if abort_recovery and not self.docked:
            self.get_logger().info('🔄 Returning to staging area (driving forward)...')
            recovery_start = time.time()
            recovery_time = 8.0  
            
            cmd = Twist()
            cmd.linear.x = 0.15 # Fixed safe recovery speed
            cmd.angular.z = 0.0
            
            while rclpy.ok() and time.time() - recovery_start < recovery_time:
                self.cmd_pub.publish(cmd)
                time.sleep(0.05)  
                
            self.cmd_pub.publish(Twist())
            self.log_to_csv(approach_speed, "N/A", "N/A", elapsed_time, "FAILED_RECOVERY")
            return

        # === Process Success Results ===
        if self.docked:
            self.record_success(approach_speed, elapsed_time)
        else:
            self.log_to_csv(approach_speed, "N/A", "N/A", elapsed_time, "FAILED_UNKNOWN")
            
    def record_success(self, approach_speed, elapsed_time):
        time.sleep(0.5) # Let TF settle
        try:
            t = self.tf_buffer.lookup_transform(
                'c310_camera_link_optical', 'tag36h11:5', rclpy.time.Time())
            
            final_x_cm = t.transform.translation.x * 100.0  
            rot = t.transform.rotation
            final_yaw = euler_from_quaternion(rot.x, rot.y, rot.z, rot.w)

            self.log_to_csv(approach_speed, final_x_cm, final_yaw, elapsed_time, "SUCCESS")
            print(f"\n✅ DOCKED! Time: {elapsed_time:.1f}s | Final X Error: {final_x_cm:.1f}cm | Final Yaw: {final_yaw:.2f}°")

        except Exception as e:
            self.get_logger().warn(f"Docked, but failed to get final TF: {e}")
            self.log_to_csv(approach_speed, "N/A", "N/A", elapsed_time, "SUCCESS_NO_TF")

    def log_to_csv(self, speed, fx, fy, t, res):
        row = [speed, 
               f"{fx:.2f}" if isinstance(fx, float) else fx,  
               f"{fy:.2f}" if isinstance(fy, float) else fy,
               f"{t:.2f}", res]
        self.csv_writer.writerow(row)
        self.csv_file.flush()  

def cli_interface(node):
    print("\n" + "="*50)
    print("🚀 MASTER SPEED DOCKING TESTER INITIALIZED 🚀")
    print(f"Data will be saved to: {node.csv_filename}")
    print("="*50 + "\n")
    
    time.sleep(1)

    while rclpy.ok():
        try:
            print("-" * 40)
            speed_input = input("Enter Approach Speed in m/s (e.g., 0.1, 0.15, 0.2) [or 'q' to quit]: ")
            if speed_input.lower() == 'q':
                print("Exiting...")
                rclpy.try_shutdown()
                break
            
            approach_speed = float(speed_input)
            
            # Enforce a sensible max/min speed for safety
            if approach_speed > 0.4 or approach_speed <= 0.0:
                print("❌ Invalid speed! Please use a realistic speed between 0.01 and 0.4 m/s.")
                continue
                
            node.execute_run(approach_speed)
            
        except ValueError:
            print("❌ Invalid input! Please enter numbers only.")
        except EOFError:
            break

def main():
    rclpy.init()
    node = ThesisDockingMaster()

    cli_thread = threading.Thread(target=cli_interface, args=(node,), daemon=True)
    cli_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.csv_file.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
