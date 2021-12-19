#! /usr/bin/env python3
from adafruit_motorkit import MotorKit
import cv2
from cv_bridge import CvBridge
import flask
import rospy
from sensor_msgs.msg import Image
import socket
import threading
import time

kit = MotorKit()
algorithm_status = False
frame = None
app = flask.Flask(__name__)

@app.route('/algorithm_status')
def get_status():
    global algorithm_status
    algorithm_status = True if flask.request.args.get('status') == 'True' else False

    time.sleep(1)
    if algorithm_status == False:
        kit.motor1.throttle = 0
        kit.motor2.throttle = 0

    return f"Succesfully set status to {algorithm_status}"

@app.route('/video_stream')
def get_stream():
    img_encoded = cv2.imencode('.jpg', frame)[1].tostring()
    return flask.Response(img_encoded, mimetype='image/jpeg')

def video_stream_publisher():
    global algorithm_status, frame
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
        if ret and algorithm_status:
            cropped_frame = frame[240:, 200:500]
            ros_image = bridge.cv2_to_imgmsg(cropped_frame, encoding="bgr8")
            video_pub.publish(ros_image)

        rate.sleep()
        
    # End video feed after shutdown
    cap.release()


if __name__ == '__main__':
    try:
        threading.Thread(target=lambda: app.run(host="0.0.0.0", port=8080)).start()
        print("SERVER STARTED")
        video_stream_publisher()
        print("VIDEO STREAMER STARTED")
    except rospy.ROSInterruptException:
        rospy.loginfo("Video streamer node terminated.")
