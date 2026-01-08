import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import glob

class VideoPublisher(Node):
    def __init__(self, video_dir):
        super().__init__('video_publisher')
        self.publisher_ = self.create_publisher(Image, '/asr_sdm_video_enhancement/input/image', 10)
        self.bridge = CvBridge()
        self.video_files = sorted(glob.glob(os.path.join(video_dir, "*.mp4")))  # Can be changed to .avi etc.
        if not self.video_files:
            self.get_logger().error(f"No video files found in {video_dir}")
            rclpy.shutdown()
            return
        self.current_video_idx = 0
        self.cap = cv2.VideoCapture(self.video_files[self.current_video_idx])
        self.get_logger().info(f"Playing {self.video_files[self.current_video_idx]}")

        timer_period = 1.0 / 30.0  # 30 FPS
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        if self.cap is None:
            return
        ret, frame = self.cap.read()
        if not ret:
            self.cap.release()
            self.current_video_idx += 1
            if self.current_video_idx < len(self.video_files):
                self.cap = cv2.VideoCapture(self.video_files[self.current_video_idx])
                self.get_logger().info(f"Playing {self.video_files[self.current_video_idx]}")
            else:
                self.get_logger().info("All videos finished")
                rclpy.shutdown()
            return

        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    video_dir = "<change_your_video_path>"  # files path
    node = VideoPublisher(video_dir)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
