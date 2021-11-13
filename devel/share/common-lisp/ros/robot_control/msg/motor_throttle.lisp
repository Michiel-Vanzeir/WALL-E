; Auto-generated. Do not edit!


(cl:in-package robot_control-msg)


;//! \htmlinclude motor_throttle.msg.html

(cl:defclass <motor_throttle> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (left_motor
    :reader left_motor
    :initarg :left_motor
    :type cl:float
    :initform 0.0)
   (right_motor
    :reader right_motor
    :initarg :right_motor
    :type cl:float
    :initform 0.0))
)

(cl:defclass motor_throttle (<motor_throttle>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <motor_throttle>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'motor_throttle)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robot_control-msg:<motor_throttle> is deprecated: use robot_control-msg:motor_throttle instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <motor_throttle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_control-msg:header-val is deprecated.  Use robot_control-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'left_motor-val :lambda-list '(m))
(cl:defmethod left_motor-val ((m <motor_throttle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_control-msg:left_motor-val is deprecated.  Use robot_control-msg:left_motor instead.")
  (left_motor m))

(cl:ensure-generic-function 'right_motor-val :lambda-list '(m))
(cl:defmethod right_motor-val ((m <motor_throttle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_control-msg:right_motor-val is deprecated.  Use robot_control-msg:right_motor instead.")
  (right_motor m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <motor_throttle>) ostream)
  "Serializes a message object of type '<motor_throttle>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'left_motor))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'right_motor))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <motor_throttle>) istream)
  "Deserializes a message object of type '<motor_throttle>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'left_motor) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'right_motor) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<motor_throttle>)))
  "Returns string type for a message object of type '<motor_throttle>"
  "robot_control/motor_throttle")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'motor_throttle)))
  "Returns string type for a message object of type 'motor_throttle"
  "robot_control/motor_throttle")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<motor_throttle>)))
  "Returns md5sum for a message object of type '<motor_throttle>"
  "810a90b3c92332e7051784244ee24b0c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'motor_throttle)))
  "Returns md5sum for a message object of type 'motor_throttle"
  "810a90b3c92332e7051784244ee24b0c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<motor_throttle>)))
  "Returns full string definition for message of type '<motor_throttle>"
  (cl:format cl:nil "Header header~%float32 left_motor~%float32 right_motor~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'motor_throttle)))
  "Returns full string definition for message of type 'motor_throttle"
  (cl:format cl:nil "Header header~%float32 left_motor~%float32 right_motor~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <motor_throttle>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <motor_throttle>))
  "Converts a ROS message object to a list"
  (cl:list 'motor_throttle
    (cl:cons ':header (header msg))
    (cl:cons ':left_motor (left_motor msg))
    (cl:cons ':right_motor (right_motor msg))
))
