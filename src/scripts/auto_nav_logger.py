import subprocess
import yaml
import sys
import time
from datetime import datetime

# 1. Define your goals here in the exact format you use in the CLI
GOALS = [
    "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 9.88842, y: 0.327861, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.999758, w: 0.0219901}}}}",
    "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 0.104701, y: -0.223352, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.0586773, w: 0.998277}}}}"
]

# The file where your AMCL coordinates and duration will be saved
OUTPUT_FILE = "amcl_saved_poses.txt"

def send_goal(pose_string):
    """Sends the goal, blocks until finished, and returns the duration in seconds."""
    cmd = f"ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \"{pose_string}\""
    
    start_time = time.perf_counter()
    subprocess.run(cmd, shell=True)
    end_time = time.perf_counter()
    
    return end_time - start_time

def get_and_save_amcl_pose(final_duration_seconds):
    """Echoes the amcl_pose once, parses the YAML, and saves X/Y + duration to a file."""
    print("\n--- Fetching FINAL AMCL pose ---")
    cmd = "ros2 topic echo /amcl_pose --once"
    
    result = subprocess.run(cmd, shell=True, capture_output=True, text=True)

    if result.returncode != 0:
        print(f"Error getting AMCL pose: {result.stderr}")
        return

    try:
        output = result.stdout
        
        # Slicing from 'header:' ensures we drop any warnings printed before the YAML
        if "header:" in output:
            output = output[output.find("header:"):]
            
        # Remove the '---' document separator that crashes PyYAML
        output = output.split('---')[0]

        # Parse the ROS 2 YAML output
        pose_data = yaml.safe_load(output)
        
        # Extract X and Y
        x = pose_data['pose']['pose']['position']['x']
        y = pose_data['pose']['pose']['position']['y']

        duration_str = f"{final_duration_seconds:.1f}s"

        # Append data to the text file
        with open(OUTPUT_FILE, "a") as f:
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            f.write(f"[{timestamp}] Final Leg Duration: {duration_str:6} | Final X: {x:.6f}, Final Y: {y:.6f}\n")

        print(f"✅ Saved to {OUTPUT_FILE} -> Final Leg Duration: {duration_str} | Final X: {x:.6f}, Final Y: {y:.6f}\n")

    except Exception as e:
        print(f"❌ Failed to parse data. Error: {e}")
        print(f"Raw Output:\n{result.stdout}")

def main():
    final_leg_time = 0.0
    
    try:
        for i, goal in enumerate(GOALS, 1):
            print(f"\n======================================")
            print(f"🚀 Executing Navigation Goal {i}/{len(GOALS)}")
            print(f"======================================")
            
            # Send the goal. We overwrite the variable each time.
            # After the loop finishes, it will hold the time of the very last goal.
            final_leg_time = send_goal(goal)
            
        # Only runs after the final goal is complete, using the time from that last leg
        get_and_save_amcl_pose(final_leg_time)
            
    except KeyboardInterrupt:
        print("\nProcess interrupted by user.")
        sys.exit(0)

if __name__ == '__main__':
    main()
