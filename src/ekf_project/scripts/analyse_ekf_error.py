import os
import matplotlib.pyplot as plt
import numpy as np
import math

# Native ROS 2 Bag Reader imports
from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions

# ROS Message types
from nav_msgs.msg import Odometry

# --- CONFIGURATION ---
BAG_PATH = os.path.expanduser('~/ros2_ws/ekf_data/final_portfolio_run')
# ---------------------

def extract_data(bag_path):
    ground_truth_x = []
    ground_truth_y = []
    est_x = []
    est_y = []
    
    gt_times = []
    est_times = []

    # Setup ROS 2 Bag Reader
    storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions(input_serialization_format='cdr',
                                         output_serialization_format='cdr')
    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    print(f"Opening Bag: {bag_path}")
    print("Processing messages...")
    
    # Counter to verify data exists
    count_gt = 0
    count_est = 0

    while reader.has_next():
        (topic, data, t) = reader.read_next()
        
        # 1. GROUND TRUTH (Now using the direct /odom_gt topic)
        if topic == '/odom_gt':
            msg = deserialize_message(data, Odometry)
            ground_truth_x.append(msg.pose.pose.position.x)
            ground_truth_y.append(msg.pose.pose.position.y)
            gt_times.append(t)
            count_gt += 1

        # 2. EKF ESTIMATE (From /odom_est topic)
        if topic == '/odom_est':
            msg = deserialize_message(data, Odometry)
            est_x.append(msg.pose.pose.position.x)
            est_y.append(msg.pose.pose.position.y)
            est_times.append(t)
            count_est += 1

    print(f"Done. Found {count_gt} Ground Truth points and {count_est} Estimates.")
    return (np.array(gt_times), np.array(ground_truth_x), np.array(ground_truth_y),
            np.array(est_times), np.array(est_x), np.array(est_y))

def calculate_error(gt_t, gt_x, gt_y, est_t, est_x, est_y):
    if len(gt_t) == 0:
        print("ERROR: No Ground Truth data found in /odom_gt.")
        return None, None
    if len(est_t) == 0:
        print("ERROR: No Estimate data found in /odom_est.")
        return None, None

    # Sort data by time (essential for interpolation)
    sort_idx_gt = np.argsort(gt_t)
    gt_t = gt_t[sort_idx_gt]
    gt_x = gt_x[sort_idx_gt]
    gt_y = gt_y[sort_idx_gt]

    # Interpolate Ground Truth to match Estimate timestamps exactly
    interp_gt_x = np.interp(est_t, gt_t, gt_x)
    interp_gt_y = np.interp(est_t, gt_t, gt_y)

    # Euclidean Distance Error (The "Engineering Proof")
    error = np.sqrt((est_x - interp_gt_x)**2 + (est_y - interp_gt_y)**2)
    return est_t, error

def main():
    if not os.path.exists(BAG_PATH):
        print(f"FOLDER NOT FOUND: {BAG_PATH}")
        return

    gt_t, gt_x, gt_y, est_t, est_x, est_y = extract_data(BAG_PATH)
    times, error = calculate_error(gt_t, gt_x, gt_y, est_t, est_x, est_y)

    if times is None:
        return

    # Normalize time to start at 0 seconds
    times = (times - times[0]) / 1e9 

    # PLOTTING
    plt.figure(figsize=(12, 8))
    
    # Plot 1: The Error Graph (The "Sawtooth")
    plt.subplot(2, 1, 1)
    plt.plot(times, error, label='EKF Position Error', color='blue', linewidth=2)
    plt.title('EKF Performance Analysis', fontsize=14)
    plt.ylabel('Error (meters)', fontsize=12)
    plt.grid(True, linestyle='--', alpha=0.7)
    plt.legend()

    # Plot 2: The Overhead Path (Top-Down View)
    plt.subplot(2, 1, 2)
    plt.plot(gt_x, gt_y, label='Ground Truth', color='orange', linestyle='--', linewidth=2)
    plt.plot(est_x, est_y, label='EKF Estimate', color='blue', linewidth=1)
    plt.title('Robot Trajectory (Top-Down)', fontsize=14)
    plt.xlabel('X Position (m)')
    plt.ylabel('Y Position (m)')
    plt.grid(True, linestyle='--', alpha=0.7)
    plt.axis('equal') # Ensure the map isn't stretched
    plt.legend()
    
    plt.tight_layout()
    
    # Save logic
    save_path = os.path.join(os.path.expanduser('~/ros2_ws/ekf_data'), 'ekf_error_analysis.png')
    plt.savefig(save_path)
    print(f"SUCCESS: Graph saved to {save_path}")
    plt.show()

if __name__ == "__main__":
    main()