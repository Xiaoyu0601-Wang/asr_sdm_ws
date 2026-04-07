#!/usr/bin/env python3
"""
Convert ROS1 bag to ROS2 bag using the rosbags library.

rosbags.rosbag1.Reader automatically normalizes ROS1 message types
to their ROS2 counterparts, so direct write to rosbags.rosbag2.Writer
produces a valid ROS2 bag.

Usage:
    python3 convert_ros1_to_ros2.py [--input INPUT_BAG] [--output OUTPUT_DIR]
"""

import os
import sys
import argparse
import shutil
from pathlib import Path

from rosbags.rosbag1 import Reader as Rosbag1Reader
from rosbags.rosbag2 import Writer as Rosbag2Writer
from rosbags.typesys import Stores, get_typestore


INPUT_BAG = "/home/lxy/asr_sdm_ws/datasheet/airground_rig_s3_2013-03-18_21-38-48.bag"
OUTPUT_DIR = "/home/lxy/asr_sdm_ws/datasheet/ros2_bags/airground_rig_s3_ros2"


def convert(input_bag: str, output_dir: str):
    src = Path(input_bag)
    dst = Path(output_dir)

    if not src.exists():
        print(f"[ERROR] Input bag not found: {src}")
        sys.exit(1)

    # Remove existing output
    if dst.exists():
        print(f"[INFO] Removing existing output: {dst}")
        shutil.rmtree(dst)

    # Create ROS2 Humble type store for ROS1→ROS2 conversion
    ts = get_typestore(Stores.ROS2_HUMBLE)

    print(f"[INFO] Reading ROS1 bag: {src}")
    with Rosbag1Reader(str(src)) as reader:
        connections = list(reader.connections)
        duration = (reader.end_time - reader.start_time) / 1e9

        print(f"[INFO] Duration: {duration:.2f}s  |  Messages: {reader.message_count}")
        print(f"[INFO] Topics ({len(connections)}):")
        for conn in connections:
            print(f"       {conn.topic}  ({conn.msgtype})")

    # Convert outside the Reader context (bag file must be closed before writing)
    msg_count = 0
    err_count = 0

    # Map: original topic name (str) -> registered Connection object
    topic_to_conn = {}

    with Rosbag2Writer(str(dst), version=9) as writer:
        for conn in connections:
            rc = writer.add_connection(conn.topic, conn.msgtype, typestore=ts)
            topic_to_conn[conn.topic] = rc

        with Rosbag1Reader(str(src)) as reader_in:
            for conn, timestamp, rawdata in reader_in.messages():
                try:
                    ros2_cdr = ts.ros1_to_cdr(rawdata, conn.msgtype)
                    writer.write(topic_to_conn[conn.topic], timestamp, ros2_cdr)
                    msg_count += 1
                except Exception as e:
                    err_count += 1
                    if err_count <= 3:
                        print(f"[WARN] {conn.topic}: {e}")

    print(f"\n[SUCCESS] Conversion complete!")
    print(f"  Input:   {src}")
    print(f"  Output:  {dst}")
    print(f"  Converted: {msg_count}  Errors: {err_count}")
    print(f"\nPlayback:")
    print(f"  ros2 bag play {dst}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Convert ROS1 bag to ROS2 bag")
    parser.add_argument("--input", "-i", default=INPUT_BAG, help="Input ROS1 bag path")
    parser.add_argument("--output", "-o", default=OUTPUT_DIR, help="Output ROS2 bag directory")
    args = parser.parse_args()
    convert(args.input, args.output)
