#!/usr/bin/env python
"""
Compute DWA performance metrics from CSV
Metrics: path length, time, smoothness, velocity stats, clearance
"""

import csv
import numpy as np
import argparse
import json
import math

class MetricsComputer:
    def __init__(self, csv_file):
        self.csv_file = csv_file
        self.data = self._load_csv()
        self.metrics = {}
    
    def _load_csv(self):
        """Load CSV file"""
        data = {
            'time': [], 'x': [], 'y': [],
            'vx': [], 'vy': [], 'omega': [],
            'linear_velocity': [], 'distance_from_start': [],
            'goal_status': []
        }
        
        try:
            with open(self.csv_file, 'r') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    data['time'].append(float(row['time']))
                    data['x'].append(float(row['x']))
                    data['y'].append(float(row['y']))
                    data['vx'].append(float(row['vx']))
                    data['vy'].append(float(row['vy']))
                    data['omega'].append(float(row['omega']))
                    data['linear_velocity'].append(float(row['linear_velocity']))
                    data['distance_from_start'].append(float(row['distance_from_start']))
                    data['goal_status'].append(row['goal_status'])
            
            return data
        except Exception as e:
            print(f"[ERROR] Failed to load CSV: {e}")
            return None
    
    def compute_path_length(self):
        """Total distance traveled"""
        x = np.array(self.data['x'])
        y = np.array(self.data['y'])
        
        dx = np.diff(x)
        dy = np.diff(y)
        distances = np.sqrt(dx**2 + dy**2)
        
        path_length = np.sum(distances)
        self.metrics['path_length_m'] = float(path_length)
        
        return path_length
    
    def compute_time_to_goal(self):
        """Time from start to goal completion"""
        times = np.array(self.data['time'])
        statuses = self.data['goal_status']
        
        # Find when goal was succeeded
        goal_time = times[-1]
        for i, status in enumerate(statuses):
            if status == "SUCCEEDED":
                goal_time = times[i]
                break
        
        self.metrics['time_to_goal_s'] = float(goal_time)
        
        return goal_time
    
    def compute_smoothness(self):
        """Trajectory smoothness - sum of angular changes"""
        x = np.array(self.data['x'])
        y = np.array(self.data['y'])
        
        # Calculate heading angles
        dx = np.diff(x)
        dy = np.diff(y)
        angles = np.arctan2(dy, dx)
        
        # Calculate angular changes
        angle_diffs = np.abs(np.diff(angles))
        # Handle wrap-around
        angle_diffs = np.minimum(angle_diffs, 2*np.pi - angle_diffs)
        
        smoothness = np.sum(angle_diffs)
        self.metrics['smoothness_rad'] = float(smoothness)
        self.metrics['avg_curvature_rad_per_m'] = float(np.mean(angle_diffs)) if len(angle_diffs) > 0 else 0
        
        return smoothness
    
    def compute_velocity_metrics(self):
        """Velocity statistics"""
        v_linear = np.array(self.data['linear_velocity'])
        omega = np.array(self.data['omega'])
        
        self.metrics['avg_linear_velocity_m_s'] = float(np.mean(v_linear))
        self.metrics['max_linear_velocity_m_s'] = float(np.max(v_linear))
        self.metrics['min_linear_velocity_m_s'] = float(np.min(v_linear))
        self.metrics['linear_velocity_std_m_s'] = float(np.std(v_linear))
        
        self.metrics['avg_angular_velocity_rad_s'] = float(np.mean(np.abs(omega)))
        self.metrics['max_angular_velocity_rad_s'] = float(np.max(np.abs(omega)))
        
        # Velocity reversals (stops and direction changes)
        velocity_sign_changes = np.sum(np.diff(np.sign(v_linear)) != 0)
        self.metrics['velocity_reversals'] = int(velocity_sign_changes)
    
    def compute_acceleration(self):
        """Acceleration metrics"""
        v_linear = np.array(self.data['linear_velocity'])
        time = np.array(self.data['time'])
        
        dv = np.diff(v_linear)
        dt = np.diff(time)
        dt = np.where(dt == 0, 1e-6, dt)  # Avoid division by zero
        
        acceleration = dv / dt
        
        self.metrics['max_acceleration_m_s2'] = float(np.max(np.abs(acceleration)))
        self.metrics['avg_acceleration_m_s2'] = float(np.mean(np.abs(acceleration)))
    
    def compute_success_metrics(self):
        """Goal achievement metrics"""
        statuses = self.data['goal_status']
        
        if "SUCCEEDED" in statuses:
            self.metrics['goal_reached'] = True
        else:
            self.metrics['goal_reached'] = False
        

    
    def compute_all(self):
        """Compute all metrics"""
        print(f"[INFO] Computing metrics from {self.csv_file}")
        
        self.compute_path_length()
        self.compute_time_to_goal()
        self.compute_smoothness()
        self.compute_velocity_metrics()
        self.compute_acceleration()
        self.compute_success_metrics()
        
        return self.metrics
    
    def save_metrics(self, output_file):
        """Save to JSON"""
        try:
            with open(output_file, 'w') as f:
                json.dump(self.metrics, f, indent=2)
            print(f"[SUCCESS] Metrics saved to {output_file}")
        except Exception as e:
            print(f"[ERROR] Failed to save metrics: {e}")
    
    def print_metrics(self):
        """Print in table format"""
        print("\n" + "="*60)
        print("DWA PERFORMANCE METRICS")
        print("="*60)
        
        # Category headers
        categories = {
            'Path': ['path_length_m', 'smoothness_rad', 'avg_curvature_rad_per_m'],
            'Time': ['time_to_goal_s'],
            'Velocity': ['avg_linear_velocity_m_s', 'max_linear_velocity_m_s', 
                        'linear_velocity_std_m_s', 'avg_angular_velocity_rad_s'],
            'Acceleration': ['max_acceleration_m_s2', 'avg_acceleration_m_s2'],
            'Success': ['goal_reached']
        }
        
        for category, keys in categories.items():
            print(f"\n{category}:")
            print("-" * 60)
            for key in keys:
                if key in self.metrics:
                    value = self.metrics[key]
                    if isinstance(value, float):
                        print(f"  {key:40s}: {value:12.4f}")
                    else:
                        print(f"  {key:40s}: {value}")
        
        print("\n" + "="*60 + "\n")

def main():
    parser = argparse.ArgumentParser(description='Compute DWA metrics')
    parser.add_argument('csv_file', help='Metrics CSV file')
    parser.add_argument('-o', '--output', help='Output JSON file', default=None)
    
    args = parser.parse_args()
    
    if args.output is None:
        args.output = args.csv_file.replace('.csv', '_metrics.json')
    
    computer = MetricsComputer(args.csv_file)
    metrics = computer.compute_all()
    
    computer.print_metrics()
    computer.save_metrics(args.output)

if __name__ == '__main__':
    main()