#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import os
import time
import cv2
import shutil
import numpy as np
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler

class MapEventHandler(FileSystemEventHandler):
    def __init__(self, node):
        self.node = node
        self.last_processed = {}

    def on_created(self, event):
        self.process(event)

    def on_modified(self, event):
        self.process(event)

    def process(self, event):
        if event.is_directory:
            return

        filename = event.src_path
        
        # Only process .pgm files (the map image)
        if not filename.endswith(".pgm"):
            return
        
        # Ignore files we just created to prevent infinite loops
        if "_cleaned.pgm" in filename:
            return

        # COOLDOWN
        now = time.time()
        if filename in self.last_processed:
            if now - self.last_processed[filename] < 2.0:
                return
        
        self.last_processed[filename] = now
        self.node.clean_map(filename)

class MapWatcherNode(Node):
    def __init__(self):
        super().__init__('map_watcher_node')
        
        # Watch the HOME directory
        default_dir = os.path.expanduser('~')
        
        self.declare_parameter('watch_directory', default_dir)
        self.watch_dir = self.get_parameter('watch_directory').value

        self.event_handler = MapEventHandler(self)
        self.observer = Observer()
        self.observer.schedule(self.event_handler, self.watch_dir, recursive=False)
        self.observer.start()

        self.get_logger().info(f'Map Watcher started. Monitoring: {self.watch_dir}')

    def clean_map(self, image_path):
        """ Creates a cleaned copy of the map instead of overwriting """
        try:
            # Wait for write to finish
            time.sleep(1.0)

            img = cv2.imread(image_path, 0)
            if img is None: return

            self.get_logger().info(f'Detected new map: {image_path}')

            # --- IMAGE PROCESSING ---
            inverted_img = cv2.bitwise_not(img)
            _, binary_img = cv2.threshold(inverted_img, 200, 255, cv2.THRESH_BINARY)
            
            kernel = np.ones((3, 3), np.uint8)
            connected_walls = cv2.morphologyEx(binary_img, cv2.MORPH_CLOSE, kernel)
            cleaned_img = cv2.medianBlur(connected_walls, 3)
            
            final_img = cv2.bitwise_not(cleaned_img)

            # --- SAVE LOGIC (UPDATED) ---
            # 1. Create new filename: mapname.pgm -> mapname_cleaned.pgm
            base_name, ext = os.path.splitext(image_path)
            new_image_path = f"{base_name}_cleaned{ext}"

            # 2. Save the Cleaned Image
            cv2.imwrite(new_image_path, final_img)
            self.get_logger().info(f'CREATED cleaned image: {os.path.basename(new_image_path)}')

            # 3. Create Matching YAML file
            # We need a new yaml file pointing to the new image, otherwise Nav2 can't load it
            original_yaml = f"{base_name}.yaml"
            new_yaml = f"{base_name}_cleaned.yaml"

            if os.path.exists(original_yaml):
                with open(original_yaml, 'r') as f_in:
                    lines = f_in.readlines()
                
                with open(new_yaml, 'w') as f_out:
                    for line in lines:
                        # Update the 'image:' line to point to the new file
                        if "image:" in line:
                            filename_only = os.path.basename(new_image_path)
                            f_out.write(f"image: {filename_only}\n")
                        else:
                            f_out.write(line)
                
                self.get_logger().info(f'CREATED cleaned yaml: {os.path.basename(new_yaml)}')

        except Exception as e:
            self.get_logger().error(f'Error cleaning map: {e}')

    def stop(self):
        self.observer.stop()
        self.observer.join()

def main(args=None):
    rclpy.init(args=args)
    node = MapWatcherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()