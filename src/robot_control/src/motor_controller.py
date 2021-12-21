#! /usr/bin/env python3
from adafruit_motorkit import MotorKit
import flask

algorithm_status = False
kit = MotorKit()

app = flask.Flask(__name__)

@app.route('/algorithm_status', methods=['POST'])
def algorithm_status():
    global algorithm_status
    
    algorithm_status = True if flask.request.form['status'] == 'True' else False
    kit.motor1.throttle, kit.motor2.throttle = 0, 0 
    return "Success"

@app.route('/motor_throttle', methods=['POST'])
def motor_throttle():
    global algorithm_status
    data = flask.request.form
    print(data)

    if algorithm_status and data['sender'] == 'video_processor':
        kit.motor1.throttle = float(data['left_motor'])
        kit.motor2.throttle = float(data['right_motor'])
        return "Success"

    if not algorithm_status and data['sender'] == 'flutter_app':
        kit.motor1.throttle = float(data['left_motor'])
        kit.motor2.throttle = float(data['right_motor'])
        return "Success"

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=8080, debug=True)