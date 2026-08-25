#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
import tf2_ros
import math, time

# Import your custom telemetry message
from nevo_msgs.msg import NevoTelemetry

def euler_from_quaternion(x, y, z, w):
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1:
        yaw_rad = math.copysign(math.pi / 2, sinp)
    else:
        yaw_rad = math.asin(sinp)
    return math.degrees(yaw_rad)

class SmartDock(Node):
    def __init__(self):
        super().__init__('smart_dock')
        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        self.docked = False
        self.current_fsm_state = 0
            
        def telemetry_cb(msg):
            self.current_fsm_state = msg.fsm_state
            if msg.is_docked and not self.docked:
                self.get_logger().info('*** LIMIT SWITCH TRIGGERED ***')
            self.docked = msg.is_docked
                
        self.create_subscription(NevoTelemetry, '/nevo_telemetry', telemetry_cb, 10)
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

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
            yaw = euler_from_quaternion(rot.x, rot.y, rot.z, rot.w)
            
            return trans, rot, yaw
            
        except Exception:
            return None, None, None

    def wait_for_nav2(self):
        for i in range(30):
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.nav_client.server_is_ready():
                self.get_logger().info(f'Nav2 server found after {i+1}s')
                return True
            if i > 0 and i % 5 == 0:
                self.get_logger().info(f'Waiting for Nav2... ({i}s)')
            time.sleep(1.0)
        self.get_logger().error('Nav2 action server not found!')
        return False

    def navigate_to(self, x, y, yaw, label='goal'):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.pose.orientation.w = math.cos(yaw / 2)

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
        # 📍 STAGING COORDINATES (UPDATE THESE NEXT)
        # ==========================================
        STAGING_X = -0.0380747
        STAGING_Y = -0.232924
        STAGING_YAW = -0.219729
        # ==========================================

        if not self.wait_for_nav2():
            return

        # === Phase 1: Idle Monitor ===
        self.get_logger().info('--- Phase 1: Idling. Waiting for NEEDS_CHARGE signal from Arduino... ---')
        while rclpy.ok() and self.current_fsm_state != 1:
            rclpy.spin_once(self, timeout_sec=0.5)
            
        self.get_logger().warn('🔋 BATTERY LOW (NEEDS_CHARGE DETECTED). Initiating autonomous docking cycle!')

        # === Phase 1.5: Navigate to Staging ===
        self.get_logger().info('--- Phase 1.5: Navigating to Staging Area ---')
        if not self.navigate_to(STAGING_X, STAGING_Y, STAGING_YAW, 'staging'):
            self.get_logger().error('Failed to reach staging area. Aborting.')
            return

        # === Phase 2: Radar sweep ===
        self.get_logger().info('--- Phase 2: Radar sweep ---')
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
            rclpy.spin_once(self, timeout_sec=0.1)
        else:
            self.get_logger().info('No tag during sweep, proceeding blindly...')
        self.cmd_pub.publish(Twist())

        # === Phase 3: Cascade Creep until 40cm Gate ===
        self.get_logger().info('--- Phase 3: Creep to Dock Entrance (40cm gate) ---')
        k_x = 0.1
        k_yaw = 0.02
        approach_speed = -0.10 
        
        creep_start = time.time()
        terminal_push_ready = False
        last_seen_time = time.time()

        while rclpy.ok() and time.time() - creep_start < 180.0 and not self.docked:
            trans, rot, yaw = self.get_tag()
            cmd = Twist()
            
            if trans is not None:
                last_seen_time = time.time()
                tag_z = abs(trans.z)
                tag_x = trans.x
                
                if tag_z <= 0.40:
                    self.get_logger().info(f'✅ 40cm line crossed! Engaging terminal push.')
                    terminal_push_ready = True
                    break
                else:
                    cmd.linear.x = approach_speed
                    cmd.angular.z = (tag_x * k_x) - (yaw * k_yaw)
            else:
                if time.time() - last_seen_time > 2.5:
                    self.get_logger().warn('⚠ Tag lost for too long. Aborting approach!')
                    break
                cmd.linear.x = -0.10

            self.cmd_pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.05)
            
        self.cmd_pub.publish(Twist())

        # === Phase 4: The Terminal Push ===
        if terminal_push_ready and not self.docked:
            self.get_logger().info('--- Phase 4: Terminal push into the funnel ---')
            push_start = time.time()
            cmd = Twist()
            cmd.linear.x = approach_speed
            cmd.angular.z = 0.0
            
            while rclpy.ok() and time.time() - push_start < 6.0 and not self.docked:
                self.cmd_pub.publish(cmd)
                rclpy.spin_once(self, timeout_sec=0.05)
                
            self.cmd_pub.publish(Twist())

        if not self.docked:
            self.get_logger().error('❌ Docking failed. Limit switch never triggered.')
            return
            
        # === Phase 5: Charging Monitor ===
        self.get_logger().info('*** DOCKED! Hardware taking over charging. Waiting for CHARGE_COMPLETE... ***')
        
        # Idle here while FSM is 2 (Wait) or 3 (Charging)
        while rclpy.ok() and self.current_fsm_state != 4:
            # If an overcurrent error triggers (5), we should probably bail out
            if self.current_fsm_state == 5:
                self.get_logger().error('❌ OVERCURRENT ERROR detected by hardware. Aborting cycle safely.')
                return
            rclpy.spin_once(self, timeout_sec=1.0)
            
        self.get_logger().info('✅ CHARGE COMPLETE. Preparing to undock.')

        # === Phase 6: Undocking ===
        self.get_logger().info('--- Phase 6: Undocking (Driving forward to clear costmap) ---')
        undock_start = time.time()
        cmd = Twist()
        cmd.linear.x = 0.15  # Drive straight forward (+X)
        cmd.angular.z = 0.0
        
        while rclpy.ok() and time.time() - undock_start < 3.0:
            self.cmd_pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.05)
            
        self.cmd_pub.publish(Twist())

        # === Phase 7: Return to Staging ===
        self.get_logger().info('--- Phase 7: Returning to Staging Area ---')
        if not self.navigate_to(STAGING_X, STAGING_Y, STAGING_YAW, 'post_charge_staging'):
            self.get_logger().error('Failed to reach staging area after charge.')
            return

        self.get_logger().info('🎉 FULL AUTONOMOUS LIFECYCLE COMPLETE! Ready for next cycle.')

def main():
    rclpy.init()
    node = SmartDock()
    node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
