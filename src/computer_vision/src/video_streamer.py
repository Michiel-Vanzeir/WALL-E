#! /usr/bin/env python3
import cv2
from cv_bridge import CvBridge
import imagezmq
import flask
import rospy
from sensor_msgs.msg import Image
import socket
import threading

algorithm_status = False
frame = None
app = flask.Flask(__name__)

@app.route('/algorithm_status', methods=['GET'])
def get_status():
    global algorithm_status
    algorithm_status = True if flask.request.args.get('status') == 'True' else False
    return "Succesfully set status to " + str(algorithm_status)

@app.route('/video_stream')
def get_stream():
    img_encoded = cv2.imencode('.jpg', frame)[1].tostring()
    return flask.Response(img_encoded, mimetype='image/jpeg')

def video_stream_publisher():
    global algorithm_status, frame
    cap = cv2.VideoCapture(0)
    rpiName = socket.gethostname()
    server_ip = "192.168.1.42"
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
        ret, frame = cap.read()
        
        # Convert the frame to a publishable ROS image and publish it 
        if ret:
            frame = frame[60:120, 0:160]
            ros_image = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            if algorithm_status: video_pub.publish(ros_image)
            sender.send_image(rpiName, frame)

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
