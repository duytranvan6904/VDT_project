#!/usr/bin/env python3
"""
Drone Camera Live Viewer Tool.
Bridges Gazebo Sim camera topics to ROS 2 and opens rqt_image_view automatically.
"""

import os
import subprocess
import sys
import time

def main():
    print("[+] Starting ROS 2 - Gazebo Sim Camera Bridge...")

    # Bridge both RGB camera (/camera) and Depth camera (/depth_camera)
    bridge_cmd = [
        "ros2", "run", "ros_gz_bridge", "parameter_bridge",
        "/camera@sensor_msgs/msg/Image@gz.msgs.Image",
        "/depth_camera@sensor_msgs/msg/Image@gz.msgs.Image",
        "/depth_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked"
    ]

    try:
        bridge_proc = subprocess.Popen(bridge_cmd)
        print("[+] Camera Bridge started successfully!")
        print("[+] Launching rqt_image_view player...")
        time.sleep(1)

        # Launch rqt_image_view
        rqt_cmd = ["ros2", "run", "rqt_image_view", "rqt_image_view"]
        rqt_proc = subprocess.Popen(rqt_cmd)
        
        rqt_proc.wait()
    except KeyboardInterrupt:
        print("\n[+] Stopping Camera Viewer...")
    finally:
        try:
            bridge_proc.terminate()
        except:
            pass

if __name__ == '__main__':
    main()
