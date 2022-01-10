#! /usr/bin/env python3
import adafruit_motorkit
import flask

algorithm_status = False
kit = adafruit_motorkit.MotorKit()

app = flask.Flask(__name__)

@app.route('/algorithm_status', methods=['GET'])
def algorithm_status():
    global algorithm_status
    
    algorithm_status = True if flask.request.args.get('status') == 'true' else False
    kit.motor1.throttle, kit.motor2.throttle = 0, 0 
    return "Succes"

@app.route('/motor_throttle', methods=['GET'])
def motor_throttle():
    global algorithm_status
    data = flask.request.args

    if algorithm_status and data['sender'] == 'video_processor':
        kit.motor1.throttle = float(data['left_motor'])
        kit.motor2.throttle = float(data['right_motor'])
        return ""

    if not algorithm_status and data['sender'] == 'flutter_app':
        kit.motor1.throttle = float(data['left_motor'])
        kit.motor2.throttle = float(data['right_motor'])
        return ""

    return ""

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=8080)
