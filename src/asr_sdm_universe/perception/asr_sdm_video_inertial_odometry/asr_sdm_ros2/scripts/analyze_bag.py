#!/usr/bin/env python3
"""
Bag Analysis and Visualization Script
Analyzes ROS2 bag files and generates visualization plots.
"""

import sys
import argparse
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
import logging

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent / 'src'))

from bag_reader_utils import BagReader, BagDataExtractor


def setup_logging(verbose=False):
    """Setup logging configuration."""
    level = logging.DEBUG if verbose else logging.INFO
    logging.basicConfig(
        level=level,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )


def analyze_bag(bag_path: str, verbose: bool = False):
    """Analyze bag file and print statistics."""
    setup_logging(verbose)
    logger = logging.getLogger(__name__)
    
    logger.info(f"Analyzing bag: {bag_path}")
    
    try:
        reader = BagReader(bag_path)
        metadata = reader.get_metadata()
        
        print("\n" + "="*60)
        print(metadata)
        print("="*60 + "\n")
        
        # Print topic details
        print("Topic Details:")
        print("-" * 60)
        for topic_name in sorted(reader.get_topic_names()):
            info = metadata.topics[topic_name]
            print(f"\n{topic_name}:")
            print(f"  Type: {info['type']}")
            print(f"  Messages: {info['message_count']}")
            print(f"  Format: {info['serialization_format']}")
        
        return reader, metadata
        
    except Exception as e:
        logger.error(f"Error analyzing bag: {e}")
        raise


def plot_imu_data(extractor: BagDataExtractor, imu_topic: str, output_file: str = None):
    """Plot IMU data."""
    print(f"\nExtracting IMU data from {imu_topic}...")
    
    imu_data = extractor.extract_imu_data(imu_topic)
    
    if imu_data['count'] == 0:
        print("No IMU data found")
        return
    
    # Convert timestamps to seconds
    timestamps = (imu_data['timestamps'] - imu_data['timestamps'][0]) / 1e9
    
    fig = plt.figure(figsize=(14, 10))
    gs = GridSpec(3, 2, figure=fig)
    
    # Linear acceleration
    ax1 = fig.add_subplot(gs[0, :])
    ax1.plot(timestamps, imu_data['linear_acceleration'][:, 0], label='X', alpha=0.7)
    ax1.plot(timestamps, imu_data['linear_acceleration'][:, 1], label='Y', alpha=0.7)
    ax1.plot(timestamps, imu_data['linear_acceleration'][:, 2], label='Z', alpha=0.7)
    ax1.set_ylabel('Linear Acceleration (m/s²)')
    ax1.set_title('IMU Linear Acceleration')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    
    # Angular velocity
    ax2 = fig.add_subplot(gs[1, :])
    ax2.plot(timestamps, imu_data['angular_velocity'][:, 0], label='X', alpha=0.7)
    ax2.plot(timestamps, imu_data['angular_velocity'][:, 1], label='Y', alpha=0.7)
    ax2.plot(timestamps, imu_data['angular_velocity'][:, 2], label='Z', alpha=0.7)
    ax2.set_ylabel('Angular Velocity (rad/s)')
    ax2.set_title('IMU Angular Velocity')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    # Acceleration magnitude
    ax3 = fig.add_subplot(gs[2, 0])
    accel_mag = np.linalg.norm(imu_data['linear_acceleration'], axis=1)
    ax3.plot(timestamps, accel_mag, color='green', alpha=0.7)
    ax3.set_ylabel('Magnitude (m/s²)')
    ax3.set_xlabel('Time (s)')
    ax3.set_title('Acceleration Magnitude')
    ax3.grid(True, alpha=0.3)
    
    # Angular velocity magnitude
    ax4 = fig.add_subplot(gs[2, 1])
    gyro_mag = np.linalg.norm(imu_data['angular_velocity'], axis=1)
    ax4.plot(timestamps, gyro_mag, color='red', alpha=0.7)
    ax4.set_ylabel('Magnitude (rad/s)')
    ax4.set_xlabel('Time (s)')
    ax4.set_title('Angular Velocity Magnitude')
    ax4.grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    if output_file:
        plt.savefig(output_file, dpi=150, bbox_inches='tight')
        print(f"Saved plot to {output_file}")
    else:
        plt.show()
    
    print(f"IMU Statistics:")
    print(f"  Total messages: {imu_data['count']}")
    print(f"  Duration: {timestamps[-1]:.2f}s")
    print(f"  Frequency: {imu_data['count'] / timestamps[-1]:.1f} Hz")
    print(f"  Accel range: [{np.min(imu_data['linear_acceleration']):.3f}, {np.max(imu_data['linear_acceleration']):.3f}]")
    print(f"  Gyro range: [{np.min(imu_data['angular_velocity']):.3f}, {np.max(imu_data['angular_velocity']):.3f}]")


def plot_trajectory(extractor: BagDataExtractor, gt_topic: str, output_file: str = None):
    """Plot trajectory data."""
    print(f"\nExtracting trajectory data from {gt_topic}...")
    
    positions = extractor.extract_positions(gt_topic)
    
    if positions['count'] == 0:
        print("No position data found")
        return
    
    # Convert timestamps to seconds
    timestamps = (positions['timestamps'] - positions['timestamps'][0]) / 1e9
    
    fig = plt.figure(figsize=(14, 10))
    gs = GridSpec(2, 2, figure=fig)
    
    # 3D trajectory
    ax1 = fig.add_subplot(gs[0, 0], projection='3d')
    ax1.plot(positions['positions'][:, 0], positions['positions'][:, 1], positions['positions'][:, 2], 'b-', alpha=0.7)
    ax1.scatter(positions['positions'][0, 0], positions['positions'][0, 1], positions['positions'][0, 2], 
               color='green', s=100, label='Start')
    ax1.scatter(positions['positions'][-1, 0], positions['positions'][-1, 1], positions['positions'][-1, 2], 
               color='red', s=100, label='End')
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_zlabel('Z (m)')
    ax1.set_title('3D Trajectory')
    ax1.legend()
    
    # XY plane
    ax2 = fig.add_subplot(gs[0, 1])
    ax2.plot(positions['positions'][:, 0], positions['positions'][:, 1], 'b-', alpha=0.7)
    ax2.scatter(positions['positions'][0, 0], positions['positions'][0, 1], color='green', s=100, label='Start')
    ax2.scatter(positions['positions'][-1, 0], positions['positions'][-1, 1], color='red', s=100, label='End')
    ax2.set_xlabel('X (m)')
    ax2.set_ylabel('Y (m)')
    ax2.set_title('XY Plane')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    ax2.axis('equal')
    
    # Position components over time
    ax3 = fig.add_subplot(gs[1, :])
    ax3.plot(timestamps, positions['positions'][:, 0], label='X', alpha=0.7)
    ax3.plot(timestamps, positions['positions'][:, 1], label='Y', alpha=0.7)
    ax3.plot(timestamps, positions['positions'][:, 2], label='Z', alpha=0.7)
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Position (m)')
    ax3.set_title('Position Components Over Time')
    ax3.legend()
    ax3.grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    if output_file:
        plt.savefig(output_file, dpi=150, bbox_inches='tight')
        print(f"Saved plot to {output_file}")
    else:
        plt.show()
    
    print(f"Trajectory Statistics:")
    print(f"  Total positions: {positions['count']}")
    print(f"  Duration: {timestamps[-1]:.2f}s")
    print(f"  Frequency: {positions['count'] / timestamps[-1]:.1f} Hz")
    print(f"  X range: [{np.min(positions['positions'][:, 0]):.3f}, {np.max(positions['positions'][:, 0]):.3f}]")
    print(f"  Y range: [{np.min(positions['positions'][:, 1]):.3f}, {np.max(positions['positions'][:, 1]):.3f}]")
    print(f"  Z range: [{np.min(positions['positions'][:, 2]):.3f}, {np.max(positions['positions'][:, 2]):.3f}]")
    
    # Calculate trajectory length
    diffs = np.diff(positions['positions'], axis=0)
    distances = np.linalg.norm(diffs, axis=1)
    total_distance = np.sum(distances)
    print(f"  Total distance: {total_distance:.3f} m")


def main():
    """Main function."""
    parser = argparse.ArgumentParser(
        description='Analyze ROS2 bag files and generate visualizations'
    )
    parser.add_argument('bag_path', help='Path to ROS2 bag directory')
    parser.add_argument('--imu-topic', default='/imu0', help='IMU topic name')
    parser.add_argument('--gt-topic', default='/leica/position', help='Ground truth topic name')
    parser.add_argument('--output-dir', help='Output directory for plots')
    parser.add_argument('--plot-imu', action='store_true', help='Plot IMU data')
    parser.add_argument('--plot-trajectory', action='store_true', help='Plot trajectory data')
    parser.add_argument('--plot-all', action='store_true', help='Plot all data')
    parser.add_argument('-v', '--verbose', action='store_true', help='Verbose output')
    
    args = parser.parse_args()
    
    # Analyze bag
    reader, metadata = analyze_bag(args.bag_path, args.verbose)
    
    # Create extractor
    extractor = BagDataExtractor(args.bag_path)
    
    # Create output directory if specified
    output_dir = None
    if args.output_dir:
        output_dir = Path(args.output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)
    
    # Plot data
    if args.plot_all or args.plot_imu:
        imu_topics = reader.get_imu_topics()
        if imu_topics:
            output_file = None
            if output_dir:
                output_file = output_dir / 'imu_data.png'
            plot_imu_data(extractor, imu_topics[0], output_file)
    
    if args.plot_all or args.plot_trajectory:
        gt_topics = reader.get_point_topics()
        if gt_topics:
            output_file = None
            if output_dir:
                output_file = output_dir / 'trajectory.png'
            plot_trajectory(extractor, gt_topics[0], output_file)


if __name__ == '__main__':
    main()

