#! /usr/bin/env python3
import cv2
from cv_bridge import CvBridge
import imagezmq
import rospy
from sensor_msgs.msg import Image, Bool
import socket

algorithm_status = False 

def setStatus(data):
    global algorithm_status
    algorithm_status = data.data

def video_stream_publisher():
    global algorithm_status
    cap = cv2.VideoCapture(0)
    rpiName = socket.gethostname()
    server_ip = socket.gethostbyname(rpiName)
    sender = imagezmq.ImageSender(connect_to=f"tcp://{server_ip}:5555")

    if not cap.isOpened():
        rospy.logerr("Unable to open camera")
        return

    cap.set(3, 160)
    cap.set(4, 120)  
    bridge = CvBridge()

    video_pub = rospy.Publisher('video_feed', Image, queue_size=2) 

    rospy.init_node('video_streamer', anonymous=True)
    rate = rospy.Rate(4) # 4hz

    while not rospy.is_shutdown():
        rospy.Subscriber('algorithm_status', Bool, callback=setStatus)
        ret, frame = cap.read()
        
        # Convert the frame to a publishable ROS image and publish it 
        if ret:
            frame = frame[60:120, 0:160]
            ros_image = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            if algorithm_status: 
                video_pub.publish(ros_image)
                rospy.loginfo("Publishing video feed")
            sender.send_image(rpiName, frame)

        rate.sleep()
        
    # End video feed after shutdown
    cap.release()


if __name__ == '__main__':
    try:
        video_stream_publisher()
    except rospy.ROSInterruptException:
        rospy.loginfo("Video streamer node terminated.")
