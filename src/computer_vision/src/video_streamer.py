#! /usr/bin/env python3
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import rospy

def video_stream():
    
    # Initialize camera feed, the cv bridge, and publisher
    cap = cv2.VideoCapture(0)
    video_pub = rospy.Publisher('video_feed', Image, queue_size=100)    
    bridge = CvBridge()

    # Check if the webcam is opened correctly
    if not cap.isOpened():
        raise IOError("Failed to open camera...")

    rospy.init_node('video_streamer')
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        frame = cv2.resize(frame,(150,150))
        image_message = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        video_pub.publish(image_message)
        rate.sleep()
        #frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        cv2.imshow('Input', frame)
        
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        video_stream()
    except rospy.ROSInterruptException:
        pass
