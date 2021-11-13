// Auto-generated. Do not edit!

// (in-package robot_control.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class motor_throttle {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.left_motor = null;
      this.right_motor = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
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
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
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
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [left_motor]
    data.left_motor = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [right_motor]
    data.right_motor = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'robot_control/motor_throttle';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '810a90b3c92332e7051784244ee24b0c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    float32 left_motor
    float32 right_motor
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new motor_throttle(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

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
