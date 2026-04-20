#!/usr/bin/env python3
"""
Analyze bogie detection density from rosbag Hesai point cloud data.
Generates Y-axis density distribution and bird's eye view plots.
"""
import sys
import struct
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
from scipy.signal import find_peaks, savgol_filter
from scipy.ndimage import gaussian_filter1d

import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2

BAG_PATH = '/home/dt/workspace/data/wuhanbei_0418_gen_10_1gou_c70/wuhanbei_0418_gen_10_1gou_c70_0.db3'
TOPIC = '/lidar_points'
DURATION_SEC = 10.0
SAMPLE_INTERVAL = 0.1  # 1 second interval
OUTPUT_DIR = '/home/dt/workspace/output'

# ROI for bogie detection (hesai_lidar frame)
# Original ROI parameters
ROI_X_MIN, ROI_X_MAX = 0.0, 2.0
ROI_Y_MIN, ROI_Y_MAX = -10.0, 1.0
ROI_Z_MIN, ROI_Z_MAX = -0.6, 0.0

Y_BINS = 600


def read_pointcloud2_xyz(msg):
    """Extract x,y,z from PointCloud2 message."""
    point_step = msg.point_step
    data = bytes(msg.data)
    n_points = msg.width * msg.height

    x_offset = y_offset = z_offset = None
    for field in msg.fields:
        if field.name == 'x':
            x_offset = field.offset
        elif field.name == 'y':
            y_offset = field.offset
        elif field.name == 'z':
            z_offset = field.offset

    if x_offset is None:
        return np.empty((0, 3))

    points = np.zeros((n_points, 3), dtype=np.float32)
    for i in range(n_points):
        base = i * point_step
        points[i, 0] = struct.unpack_from('f', data, base + x_offset)[0]
        points[i, 1] = struct.unpack_from('f', data, base + y_offset)[0]
        points[i, 2] = struct.unpack_from('f', data, base + z_offset)[0]

    mask = np.isfinite(points).all(axis=1)
    return points[mask]


def read_bag_frames(bag_path, topic, duration_sec):
    """Read point cloud frames from rosbag for the given duration."""
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr')

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    storage_filter = rosbag2_py.StorageFilter(topics=[topic])
    reader.set_filter(storage_filter)

    frames = []
    start_time = None

    while reader.has_next():
        topic_name, data, timestamp = reader.read_next()
        if start_time is None:
            start_time = timestamp
        elapsed = (timestamp - start_time) / 1e9
        if elapsed > duration_sec:
            break

        msg = deserialize_message(data, PointCloud2)
        points = read_pointcloud2_xyz(msg)
        frames.append((elapsed, points))

    return frames


def filter_roi(points):
    """Filter points to ROI and exclude invalid (0,0,0) points."""
    mask = (
        (points[:, 0] >= ROI_X_MIN) & (points[:, 0] <= ROI_X_MAX) &
        (points[:, 1] >= ROI_Y_MIN) & (points[:, 1] <= ROI_Y_MAX) &
        (points[:, 2] >= ROI_Z_MIN) & (points[:, 2] <= ROI_Z_MAX)
    )
    # Exclude invalid points (0,0,0)
    valid_mask = (points[:, 0] != 0.0) | (points[:, 1] != 0.0) | (points[:, 2] != 0.0)
    return points[mask & valid_mask]


def compute_y_density(points, y_min, y_max, n_bins):
    """Compute point density along Y axis."""
    bin_edges = np.linspace(y_min, y_max, n_bins + 1)
    bin_centers = (bin_edges[:-1] + bin_edges[1:]) / 2
    density, _ = np.histogram(points[:, 1], bins=bin_edges)
    return bin_centers, density.astype(float)


def detect_peaks_and_valleys(y, density, prominence_factor=0.3):
    """Detect peaks (bogie centers) and valleys (decoupling points).
    
    Returns:
        peaks: indices of peak positions
        valleys: indices of valley positions (between peaks)
    """
    # Smooth the density signal
    smoothed = gaussian_filter1d(density, sigma=3)
    
    # Find peaks with prominence
    mean_density = np.mean(smoothed[smoothed > 0])
    std_density = np.std(smoothed[smoothed > 0])
    height_threshold = mean_density + 0.5 * std_density
    
    peaks, properties = find_peaks(smoothed, 
                                  height=height_threshold,
                                  distance=20,  # minimum distance between peaks (about 0.3m)
                                  prominence=std_density * prominence_factor)
    
    # Find valleys between peaks
    valleys = []
    for i in range(len(peaks) - 1):
        start = peaks[i]
        end = peaks[i + 1]
        valley_idx = start + np.argmin(smoothed[start:end])
        valleys.append(valley_idx)
    
    return peaks, valleys, smoothed


def detect_decoupling_points(y, density, 
                             bogie_length_min=2.6, bogie_length_max=3.9,
                             gap_width_min=1.5, gap_width_max=3.0):
    """Detect bogies and decoupling points based on density peaks and valleys.
    
    A valid bogie pair has:
    - Bogie length between bogie_length_min and bogie_length_max
    - Gap between bogies between gap_width_min and gap_width_max
    """
    peaks, valleys, smoothed = detect_peaks_and_valleys(y, density)
    
    bin_width = y[1] - y[0] if len(y) > 1 else 0.1
    decoupling_points = []
    
    for valley_idx in valleys:
        # Check if this is a valid gap
        if valley_idx > 0 and valley_idx < len(y) - 1:
            gap_y = y[valley_idx]
            # Find adjacent peaks
            left_peak = None
            right_peak = None
            for p in peaks:
                if p < valley_idx:
                    left_peak = p
                elif p > valley_idx and right_peak is None:
                    right_peak = p
                    break
            
            if left_peak is not None and right_peak is not None:
                gap_width = y[right_peak] - y[left_peak]
                if gap_width_min <= gap_width <= gap_width_max:
                    decoupling_points.append((gap_y, gap_width))
    
    return peaks, valleys, smoothed, decoupling_points


def select_frames_by_interval(frames, interval_sec):
    """Select frames closest to each interval_sec timestamp."""
    if not frames:
        return []
    
    selected = []
    target_time = 0.0
    
    while target_time <= DURATION_SEC:
        best_idx = 0
        best_diff = float('inf')
        for i, (elapsed, _) in enumerate(frames):
            diff = abs(elapsed - target_time)
            if diff < best_diff:
                best_diff = diff
                best_idx = i
        selected.append((best_idx, target_time))
        target_time += interval_sec
    
    return selected


def main():
    import os
    import shutil
    
    # 清理输出文件夹
    if os.path.exists(OUTPUT_DIR):
        print(f"Cleaning output directory: {OUTPUT_DIR}")
        shutil.rmtree(OUTPUT_DIR)
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    print(f"Reading bag: {BAG_PATH}")
    print(f"Topic: {TOPIC}, Duration: {DURATION_SEC}s, Sample interval: {SAMPLE_INTERVAL}s")
    frames = read_bag_frames(BAG_PATH, TOPIC, DURATION_SEC)
    print(f"Read {len(frames)} frames")

    selected_frames = select_frames_by_interval(frames, SAMPLE_INTERVAL)
    print(f"Selected {len(selected_frames)} frames at {SAMPLE_INTERVAL}s intervals")

    for frame_idx, target_time in selected_frames:
        elapsed, points = frames[frame_idx]
        roi_points = filter_roi(points)
        
        # Export CSV for every frame
        csv_path = os.path.join(OUTPUT_DIR, f'frame_{frame_idx:04d}_t{target_time:.1f}s_roi.csv')
        np.savetxt(csv_path, roi_points, delimiter=',', fmt='%.6f',
                   header='x,y,z', comments='')
        print(f"Exported CSV: {csv_path} ({len(roi_points)} points)")
        print(f"Frame {frame_idx}: t={elapsed:.2f}s (target={target_time:.1f}s), total={len(points)}, roi={len(roi_points)}")

        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(16, 10),
            gridspec_kw={'height_ratios': [1, 2]})

        # Top: Y-axis density with peak detection
        if len(roi_points) > 0:
            bin_centers, density = compute_y_density(roi_points, ROI_Y_MIN, ROI_Y_MAX, Y_BINS)
            ax1.plot(bin_centers, density, 'b-', linewidth=0.8, alpha=0.5, label='Raw')
            
            # Detect peaks and valleys
            peaks, valleys, smoothed = detect_peaks_and_valleys(bin_centers, density)
            
            # Plot smoothed density
            ax1.plot(bin_centers, smoothed, 'b-', linewidth=1.2, label='Smoothed')
            
            # Plot peaks (bogie positions)
            if len(peaks) > 0:
                peak_x = bin_centers[peaks]
                peak_y = smoothed[peaks]
                ax1.plot(peak_x, peak_y, 'rv', markersize=8, label=f'Bogies ({len(peaks)})')
                
                # Annotate peaks
                for i, (px, py) in enumerate(zip(peak_x, peak_y)):
                    ax1.annotate(f'{px:.1f}m', (px, py), textcoords="offset points", 
                               xytext=(0, 10), ha='center', fontsize=8)
            
            # Plot valleys (decoupling points)
            if len(valleys) > 0:
                valley_x = bin_centers[valleys]
                valley_y = smoothed[valleys]
                ax1.plot(valley_x, valley_y, 'g^', markersize=8, label=f'Gaps ({len(valleys)})')
                
                # Annotate decoupling points
                for vx, vy in zip(valley_x, valley_y):
                    ax1.annotate(f'{vx:.1f}m', (vx, vy), textcoords="offset points",
                               xytext=(0, -15), ha='center', fontsize=8, color='green')
            
            # Auto-scale ignoring top 1% outliers
            sorted_d = np.sort(smoothed[smoothed > 0])
            if len(sorted_d) > 0:
                ylim_top = sorted_d[int(len(sorted_d) * 0.99)] * 1.5
                ax1.set_ylim(0, max(ylim_top, 10))
            
            ax1.legend(loc='upper right', fontsize=8)
        ax1.set_ylabel('Point count')
        ax1.set_title(f'Y-axis density (ROI x=[{ROI_X_MIN},{ROI_X_MAX}]m, z=[{ROI_Z_MIN},{ROI_Z_MAX}]m)')
        ax1.grid(True, alpha=0.3)
        ax1.set_xlim(ROI_Y_MIN, ROI_Y_MAX)

        # Bottom: YOZ projection (Y horizontal, Z vertical)
        if len(roi_points) > 0:
            sc = ax2.scatter(roi_points[:, 1], roi_points[:, 2],
                           s=0.3, c=roi_points[:, 0], cmap='viridis',
                           vmin=ROI_X_MIN, vmax=ROI_X_MAX)
            plt.colorbar(sc, ax=ax2, label='X (m)', shrink=0.6)
        ax2.set_xlabel('Y (m) - hesai_lidar frame')
        ax2.set_ylabel('Z (m)')
        ax2.set_title('YOZ projection (ROI) [Y: 1m/div, Z: 0.1m/div]')
        ax2.set_xlim(ROI_Y_MIN, ROI_Y_MAX)
        # Expand Z axis display range to compress appearance (multiply range by 2)
        z_center = (ROI_Z_MIN + ROI_Z_MAX) / 2
        z_half_range = (ROI_Z_MAX - ROI_Z_MIN) / 2 * 2
        ax2.set_ylim(z_center - z_half_range, z_center + z_half_range)
        ax2.grid(True, alpha=0.3)

        fig.suptitle(f't = {elapsed:.2f}s  (frame {frame_idx})', fontsize=14)
        fig.tight_layout()

        out_path = os.path.join(OUTPUT_DIR, f'frame_{frame_idx:04d}_t{target_time:.1f}s.png')
        fig.savefig(out_path, dpi=150)
        plt.close(fig)
        print(f"  Saved: {out_path}")

    print("Done.")


if __name__ == '__main__':
    main()
