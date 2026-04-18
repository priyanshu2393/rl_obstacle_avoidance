#!/usr/bin/env python
"""
Plot DWA performance metrics
Creates visualization of trajectory and velocity profiles
"""

import csv
import numpy as np
import matplotlib.pyplot as plt
import argparse
import json

class MetricsPlotter:
    def __init__(self, csv_file):
        self.csv_file = csv_file
        self.data = self._load_csv()
        plt.style.use('seaborn-v0_8-darkgrid')
    
    def _load_csv(self):
        """Load CSV file"""
        data = {
            'time': [], 'x': [], 'y': [],
            'vx': [], 'vy': [], 'omega': [],
            'linear_velocity': [], 'distance_from_start': []
        }
        
        try:
            with open(self.csv_file, 'r') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    for key in data.keys():
                        data[key].append(float(row[key]))
            return data
        except Exception as e:
            print(f"[ERROR] Failed to load CSV: {e}")
            return None
    
    def plot_trajectory(self, output_file=None):
        """Plot robot trajectory on map"""
        fig, ax = plt.subplots(figsize=(10, 8))
        
        x = np.array(self.data['x'])
        y = np.array(self.data['y'])
        
        # Plot trajectory
        ax.plot(x, y, 'b-', linewidth=2, alpha=0.7, label='Path')
        
        # Mark start and goal
        ax.plot(x[0], y[0], 'go', markersize=12, label='Start', zorder=5)
        ax.plot(x[-1], y[-1], 'r*', markersize=15, label='Goal', zorder=5)
        
        # Add arrows to show direction
        for i in range(0, len(x), max(1, len(x)//10)):
            if i + 1 < len(x):
                dx = x[i+1] - x[i]
                dy = y[i+1] - y[i]
                ax.arrow(x[i], y[i], dx*5, dy*5, head_width=0.1, head_length=0.1, 
                        fc='blue', ec='blue', alpha=0.5)
        
        ax.set_xlabel('X (meters)', fontsize=11)
        ax.set_ylabel('Y (meters)', fontsize=11)
        ax.set_title('DWA Robot Trajectory', fontsize=13, fontweight='bold')
        ax.legend(fontsize=10)
        ax.grid(True, alpha=0.3)
        ax.axis('equal')
        
        if output_file:
            plt.savefig(output_file, dpi=300, bbox_inches='tight')
            print(f"[SUCCESS] Trajectory plot saved: {output_file}")
        
        return fig
    
    def plot_velocity_profiles(self, output_file=None):
        """Plot velocity over time"""
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))
        
        time = np.array(self.data['time'])
        v_linear = np.array(self.data['linear_velocity'])
        omega = np.array(self.data['omega'])
        
        # Linear velocity
        ax1.plot(time, v_linear, 'b-', linewidth=2, label='Linear Velocity')
        ax1.fill_between(time, v_linear, alpha=0.3)
        ax1.set_xlabel('Time (s)', fontsize=11)
        ax1.set_ylabel('Linear Velocity (m/s)', fontsize=11)
        ax1.set_title('Linear Velocity Profile', fontsize=12, fontweight='bold')
        ax1.grid(True, alpha=0.3)
        ax1.legend(fontsize=10)
        
        # Angular velocity
        ax2.plot(time, omega, 'r-', linewidth=2, label='Angular Velocity')
        ax2.axhline(y=0, color='k', linestyle='--', alpha=0.3)
        ax2.set_xlabel('Time (s)', fontsize=11)
        ax2.set_ylabel('Angular Velocity (rad/s)', fontsize=11)
        ax2.set_title('Angular Velocity Profile', fontsize=12, fontweight='bold')
        ax2.grid(True, alpha=0.3)
        ax2.legend(fontsize=10)
        
        plt.tight_layout()
        
        if output_file:
            plt.savefig(output_file, dpi=300, bbox_inches='tight')
            print(f"[SUCCESS] Velocity profiles saved: {output_file}")
        
        return fig
    
    def plot_distance_vs_time(self, output_file=None):
        """Plot distance from start vs time"""
        fig, ax = plt.subplots(figsize=(10, 6))
        
        time = np.array(self.data['time'])
        distance = np.array(self.data['distance_from_start'])
        
        ax.plot(time, distance, 'g-', linewidth=2.5, label='Distance from Start')
        ax.fill_between(time, distance, alpha=0.3, color='green')
        
        ax.set_xlabel('Time (s)', fontsize=11)
        ax.set_ylabel('Distance (meters)', fontsize=11)
        ax.set_title('Robot Distance from Start vs Time', fontsize=12, fontweight='bold')
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=10)
        
        if output_file:
            plt.savefig(output_file, dpi=300, bbox_inches='tight')
            print(f"[SUCCESS] Distance plot saved: {output_file}")
        
        return fig
    
    def plot_acceleration(self, output_file=None):
        """Plot acceleration profile"""
        fig, ax = plt.subplots(figsize=(10, 6))
        
        v_linear = np.array(self.data['linear_velocity'])
        time = np.array(self.data['time'])
        
        dv = np.diff(v_linear)
        dt = np.diff(time)
        dt = np.where(dt == 0, 1e-6, dt)
        acceleration = dv / dt
        time_accel = time[:-1]
        
        ax.plot(time_accel, acceleration, 'purple', linewidth=2, label='Acceleration')
        ax.axhline(y=0, color='k', linestyle='--', alpha=0.3)
        ax.fill_between(time_accel, acceleration, alpha=0.3, color='purple')
        
        ax.set_xlabel('Time (s)', fontsize=11)
        ax.set_ylabel('Acceleration (m/s²)', fontsize=11)
        ax.set_title('Robot Acceleration Profile', fontsize=12, fontweight='bold')
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=10)
        
        if output_file:
            plt.savefig(output_file, dpi=300, bbox_inches='tight')
            print(f"[SUCCESS] Acceleration plot saved: {output_file}")
        
        return fig
    
    def plot_all(self, output_dir='.'):
        """Generate all plots"""
        self.plot_trajectory(f'{output_dir}/01_trajectory.png')
        self.plot_velocity_profiles(f'{output_dir}/02_velocity_profiles.png')
        self.plot_distance_vs_time(f'{output_dir}/03_distance_vs_time.png')
        self.plot_acceleration(f'{output_dir}/04_acceleration.png')

def main():
    parser = argparse.ArgumentParser(description='Plot DWA metrics')
    parser.add_argument('csv_file', help='Metrics CSV file')
    parser.add_argument('-o', '--output-dir', default='.', help='Output directory')
    
    args = parser.parse_args()
    
    plotter = MetricsPlotter(args.csv_file)
    plotter.plot_all(args.output_dir)
    
    print(f"\n[SUCCESS] All plots generated in {args.output_dir}/")

if __name__ == '__main__':
    main()