#! /usr/bin/env python3
import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

def video_stream():
    
    # Initialize camera, publisher, and the CvBridge
    cap = cv2.VideoCapture(0)
    cap.set(3, 160)
    cap.set(4, 120)
    video_pub = rospy.Publisher('video_feed', Image, queue_size=10)    
    bridge = CvBridge()

    # Check whether the initialization of the camera went well
    if not cap.isOpened():
        raise IOError("Failed to open camera in video_streamer.py")

    # Start the node, publish three times per second
    rospy.init_node('video_streamer')
    rate = rospy.Rate(4) # 4hz

    while not rospy.is_shutdown():
        # Read the frame from the video feed and crop it
        ret, frame = cap.read()
        frame = frame[60:120, 0:160]
        
        # Convert the frame to a ROS image and publish it 
        image_message = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        video_pub.publish(image_message)
        rospy.loginfo("Published video frame");      
        rate.sleep()
        
    # End video feed after shutdown
    cap.release()

if __name__ == '__main__':
    try:
        video_stream()
    except rospy.ROSInterruptException:
        pass
