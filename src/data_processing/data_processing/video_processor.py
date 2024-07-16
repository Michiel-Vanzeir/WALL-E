import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from cv_bridge import CvBridge
import cv2
import numpy as np

class VideoProcessor(Node):
    def __init__(self):
        super().__init__('video_processor')
        self.subscription = self.create_subscription(
            Image,
            '/videofeed',
            self.listener_callback,
            2)
        self.subscription  # prevent unused variable warning

        self.publisher = self.create_publisher(Float64, 'line_location', 2)

        self.cv_bridge = CvBridge()

    def listener_callback(self, msg):
        frame = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Smoothes the frame to reduce noise
        blurred_frame = cv2.GaussianBlur(src=gray_frame, ksize=(5,5), sigmaX=1.5)

        # Dark regions become 1, light regions 0
        _, binary_frame = cv2.threshold(blurred_frame, 50, 255, cv2.THRESH_BINARY_INV)

        contours, _ = cv2.findContours(binary_frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Finds the contour with the largest area, probably the line
        largest_contour = max(contours, key=cv2.contourArea, default=None)

        if largest_contour is not None: 
            mean_x = np.mean([point[0][0] for point in largest_contour])

            msg = Float64()
            msg.data = mean_x
            self.publisher.publish(msg)





def main(args=None):
    rclpy.init(args=args)

    videofeed_sub = VideoProcessor()

    rclpy.spin(videofeed_sub)

    videofeed_sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()