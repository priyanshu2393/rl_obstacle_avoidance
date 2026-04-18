#!/usr/bin/env python
"""
Record DWA performance metrics in real-time
Saves to CSV file
"""

import rospy
import csv
import os
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import GoalStatusArray
import math

class MetricsRecorder:
    def __init__(self, output_file):
        self.output_file = output_file
        self.csv_file = open(output_file, 'w', newline='')
        self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=[
            'time', 'x', 'y', 'vx', 'vy', 'omega',
            'linear_velocity', 'distance_from_start',
            'goal_status'
        ])
        self.csv_writer.writeheader()
        
        # Initial position for distance calculation
        self.start_x = None
        self.start_y = None
        self.prev_x = None
        self.prev_y = None
        
        # Subscribers
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback) # type: ignore
        self.status_sub = rospy.Subscriber('/move_base/status', GoalStatusArray, self.status_callback)
        
        self.goal_status = "IDLE"
        self.start_time = rospy.Time.now()
        
        print(f"[INFO] Recording metrics to {output_file}")
    
    def odom_callback(self, msg):
        """Process odometry data"""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        omega = msg.twist.twist.angular.z
        
        # Set initial position
        if self.start_x is None:
            self.start_x = x
            self.start_y = y
        
        # Calculate metrics
        linear_velocity = math.sqrt(vx**2 + vy**2)
        distance_from_start = math.sqrt((x - self.start_x)**2 + (y - self.start_y)**2)
        
        # Get elapsed time
        elapsed_time = (rospy.Time.now() - self.start_time).to_sec()
        
        # Write to CSV
        self.csv_writer.writerow({
            'time': elapsed_time,
            'x': x,
            'y': y,
            'vx': vx,
            'vy': vy,
            'omega': omega,
            'linear_velocity': linear_velocity,
            'distance_from_start': distance_from_start,
            'goal_status': self.goal_status
        })
        
        self.csv_file.flush()
    
    def status_callback(self, msg):
        if not msg.status_list:
            return
        print("Current status list:", [s.status for s in msg.status_list])

        for status_msg in reversed(msg.status_list):
            status = status_msg.status

            if status == 3:  # SUCCEEDED
                self.goal_status = "SUCCEEDED"
                return
            elif status == 1:  # ACTIVE
                self.goal_status = "ACTIVE"
                return
            elif status == 4:
                self.goal_status = "ABORTED"
                return

    
    def stop(self):
        """Stop recording"""
        self.csv_file.close()
        print(f"[SUCCESS] Metrics saved to {self.output_file}")

def main():
    import argparse
    parser = argparse.ArgumentParser(description='Record DWA metrics')
    parser.add_argument('-o', '--output', default='metrics.csv', help='Output CSV file')
    args = parser.parse_args()
    
    rospy.init_node('metrics_recorder', anonymous=True)
    recorder = MetricsRecorder(args.output)
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("\n[INFO] Stopping recording...")
        recorder.stop()
    
if __name__ == '__main__':
    main()