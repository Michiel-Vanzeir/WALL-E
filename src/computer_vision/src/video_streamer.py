#! /usr/bin/env python3
from adafruit_motorkit import MotorKit
import cv2
from cv_bridge import CvBridge
import rospy
from sensor_msgs.msg import Image


def video_stream_publisher():
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        rospy.logerr("Unable to open camera")
        return

    cap.set(3, 640)
    cap.set(4, 480)  
    bridge = CvBridge()

    video_pub = rospy.Publisher('video_feed', Image, queue_size=2) 

    rospy.init_node('video_streamer', anonymous=True)
    rate = rospy.Rate(4) # 4hz
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        
        # Convert the frame to a publishable ROS image and publish it 
        if ret:
            cropped_frame = frame[240:, 200:500]
            ros_image = bridge.cv2_to_imgmsg(cropped_frame, encoding="bgr8")
            video_pub.publish(ros_image)

        rate.sleep()
        
    # End video feed after shutdown
    cap.release()


if __name__ == '__main__':
    try:
        rospy.loginfo("Video streamer node started.")
        video_stream_publisher()
    except rospy.ROSInterruptException:
        rospy.loginfo("Video streamer node terminated.")
