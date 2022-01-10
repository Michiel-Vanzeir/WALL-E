#! /usr/bin/env python3
import cv2
import cv_bridge
import rospy
from sensor_msgs.msg import Image


def video_stream_publisher():
    cap = cv2.VideoCapture(0)
    bridge = cv_bridge.CvBridge()

    if not cap.isOpened():
        rospy.logerr("Unable to open camera")
        return

    cap.set(3, 160)
    cap.set(4, 120)  

    video_pub = rospy.Publisher('video_feed', Image, queue_size=2) 
    rospy.init_node('video_streamer', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        
        # Convert the frame to a ROS image and publish it 
        if ret:
            frame = frame[60:100, :160]
            ros_image = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            video_pub.publish(ros_image)

        rate.sleep()
        
    # Close the capture
    cap.release()


if __name__ == '__main__':
    try:
        video_stream_publisher()
    except rospy.ROSInterruptException:
        rospy.loginfo("Video streamer node terminated.")
