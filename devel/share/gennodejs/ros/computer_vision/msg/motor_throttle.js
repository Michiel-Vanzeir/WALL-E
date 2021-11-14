// Auto-generated. Do not edit!

// (in-package computer_vision.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class motor_throttle {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.left_motor = null;
      this.right_motor = null;
    }
    else {
      if (initObj.hasOwnProperty('left_motor')) {
        this.left_motor = initObj.left_motor
      }
      else {
        this.left_motor = 0.0;
      }
      if (initObj.hasOwnProperty('right_motor')) {
        this.right_motor = initObj.right_motor
      }
      else {
        this.right_motor = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type motor_throttle
    // Serialize message field [left_motor]
    bufferOffset = _serializer.float32(obj.left_motor, buffer, bufferOffset);
    // Serialize message field [right_motor]
    bufferOffset = _serializer.float32(obj.right_motor, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type motor_throttle
    let len;
    let data = new motor_throttle(null);
    // Deserialize message field [left_motor]
    data.left_motor = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [right_motor]
    data.right_motor = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'computer_vision/motor_throttle';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3e3717ac8e9443aa62d7102a5860f5e7';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 left_motor
    float32 right_motor
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new motor_throttle(null);
    if (msg.left_motor !== undefined) {
      resolved.left_motor = msg.left_motor;
    }
    else {
      resolved.left_motor = 0.0
    }

    if (msg.right_motor !== undefined) {
      resolved.right_motor = msg.right_motor;
    }
    else {
      resolved.right_motor = 0.0
    }

    return resolved;
    }
};

module.exports = motor_throttle;
