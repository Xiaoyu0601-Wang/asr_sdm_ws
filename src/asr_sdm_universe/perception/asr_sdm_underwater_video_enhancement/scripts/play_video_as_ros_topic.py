import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import glob
import argparse

class VideoPublisher(Node):
    def __init__(self, path):
        super().__init__('video_publisher')
        self.publisher_ = self.create_publisher(Image, '/asr_sdm_underwater_video_enhancement/input/image', 10)
        self.bridge = CvBridge()
        VIDEO_EXTS = (".mp4", ".avi", ".mkv", ".mov")

        if os.path.isfile(path):
            self.video_files = [path]
        else:
            self.video_files = sorted(
                f for f in glob.glob(os.path.join(path, "*"))
                if f.lower().endswith(VIDEO_EXTS)
)

        if not self.video_files:
            self.get_logger().error(f"No video files found in {path}")
            rclpy.shutdown()
            return

        self.current_video_idx = 0
        self.cap = cv2.VideoCapture(self.video_files[0])

        self.get_logger().info(f"Playing {self.video_files[0]}")

        self.finished = False
        timer_period = 1.0 / 30.0  # 30 FPS
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()

        if not ret:
            self.cap.release()
            self.current_video_idx += 1

            if self.current_video_idx < len(self.video_files):
                # play the next video
                next_video = self.video_files[self.current_video_idx]
                self.get_logger().info(f"Playing {next_video}")
                self.cap = cv2.VideoCapture(next_video)
                return
            else:
                # play all videos finished
                self.get_logger().info("All videos finished")
                self.timer.cancel()
                self.finished = True
                return

        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.publisher_.publish(msg)

def main(args=None):

    parser = argparse.ArgumentParser()
    parser.add_argument("file_path")

    parsed_args, ros_args = parser.parse_known_args(args=args)

    rclpy.init(args=ros_args)

    node = VideoPublisher(parsed_args.file_path)
    while rclpy.ok() and not node.finished:
        rclpy.spin_once(node, timeout_sec=0.1)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
