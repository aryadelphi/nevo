import rclpy
from rclpy.node import Node
import tf2_ros
import numpy as np
import csv
import time
import math

# Standard ROS 2 AprilTag message
from apriltag_msgs.msg import AprilTagDetectionArray  

class TagEvaluator(Node):
    def __init__(self):
        super().__init__('tag_evaluator')

        # --- Parameters ---
        self.declare_parameter('mode', 'fov') 
        self.declare_parameter('test_distance', 80.0) # NEW: Distance from camera to wall (cm)
        self.declare_parameter('test_offset', 0.0)    # CHANGED: Left/Right offset (cm)
        self.declare_parameter('interval', 5.0)       
        self.declare_parameter('iterations', 6)       
        
        self.mode = self.get_parameter('mode').value
        self.test_distance = self.get_parameter('test_distance').value
        self.test_offset = self.get_parameter('test_offset').value
        self.interval = self.get_parameter('interval').value
        self.max_iterations = self.get_parameter('iterations').value

        # --- Frames & Topics ---
        self.camera_frame = 'c310_camera_link_optical' 
        self.tag_frame = 'tag36h11:5'
        self.detection_topic = '/detections'
        
        # --- State Tracking ---
        self.current_iteration = 0
        
        # --- Data Storage ---
        self.reset_data_buffers()
        
        # --- TF Setup ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # --- Subscribers ---
        self.sub = self.create_subscription(
            AprilTagDetectionArray,  
            self.detection_topic,  
            self.detection_callback,  
            10
        )

        # --- Timers ---
        self.tf_timer = self.create_timer(0.033, self.record_tf) # ~30Hz polling
        self.interval_start_time = time.time()
        
        self.get_logger().info(
            f"Starting {self.mode.upper()} test: Distance={self.test_distance}cm, Offset={self.test_offset}cm. "
            f"Taking {self.max_iterations} samples of {self.interval}s each."
        )

    def reset_data_buffers(self):
        self.detection_times = []
        self.translations = {'x': [], 'y': [], 'z': []}
        self.rotations = {'roll': [], 'pitch': [], 'yaw': []}

    def detection_callback(self, msg):
        if len(msg.detections) > 0:
            self.detection_times.append(time.time())

    def record_tf(self):
        current_time = time.time()
        
        if current_time - self.interval_start_time >= self.interval:
            self.current_iteration += 1
            self.process_and_save_data(self.current_iteration)
            
            if self.current_iteration >= self.max_iterations:
                self.get_logger().info("All iterations complete. Shutting down.")
                self.tf_timer.cancel()
                rclpy.shutdown()
                return
            else:
                self.reset_data_buffers()
                self.interval_start_time = time.time()

        try:
            trans = self.tf_buffer.lookup_transform(
                self.camera_frame,  
                self.tag_frame,  
                rclpy.time.Time()
            )
            
            self.translations['x'].append(trans.transform.translation.x)
            self.translations['y'].append(trans.transform.translation.y)
            self.translations['z'].append(trans.transform.translation.z)
            
            q = trans.transform.rotation
            roll, pitch, yaw = self.euler_from_quaternion(q.x, q.y, q.z, q.w)
            
            self.rotations['roll'].append(roll)
            self.rotations['pitch'].append(pitch)
            self.rotations['yaw'].append(yaw)
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass 

    def euler_from_quaternion(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
        
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
        
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
        
        return math.degrees(roll_x), math.degrees(pitch_y), math.degrees(yaw_z)

    def process_and_save_data(self, iteration_num):
        hz = 0.0
        if len(self.detection_times) > 1:
            total_time = self.detection_times[-1] - self.detection_times[0]
            hz = len(self.detection_times) / total_time
            
        std_x = np.std(self.translations['x']) if self.translations['x'] else 0.0
        std_y = np.std(self.translations['y']) if self.translations['y'] else 0.0
        std_z = np.std(self.translations['z']) if self.translations['z'] else 0.0
        
        std_roll = np.std(self.rotations['roll']) if self.rotations['roll'] else 0.0
        std_pitch = np.std(self.rotations['pitch']) if self.rotations['pitch'] else 0.0
        std_yaw = np.std(self.rotations['yaw']) if self.rotations['yaw'] else 0.0
        
        filename = f"{self.mode}_test_results.csv"
        
        file_exists = False
        try:
            with open(filename, 'r'):
                file_exists = True
        except FileNotFoundError:
            pass
            
        with open(filename, 'a', newline='') as f:
            writer = csv.writer(f)
            if not file_exists:
                # NEW HEADER: Records both Distance and Offset
                writer.writerow(['Mode', 'Distance(cm)', 'Offset(cm)', 'Iteration', 'Detection_Hz', 'Std_X(m)', 'Std_Y(m)', 'Std_Z(m)', 'Std_Roll(deg)', 'Std_Pitch(deg)', 'Std_Yaw(deg)'])
                
            writer.writerow([
                self.mode, self.test_distance, self.test_offset, iteration_num, round(hz, 2),  
                round(std_x, 5), round(std_y, 5), round(std_z, 5),  
                round(std_roll, 3), round(std_pitch, 3), round(std_yaw, 3)
            ])
            
        self.get_logger().info(f"Iteration {iteration_num}/{self.max_iterations} saved -> Hz: {hz:.2f} | Z-Jitter: {std_z:.5f}m")

def main(args=None):
    rclpy.init(args=args)
    node = TagEvaluator()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
