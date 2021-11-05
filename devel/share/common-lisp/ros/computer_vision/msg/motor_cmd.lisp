; Auto-generated. Do not edit!


(cl:in-package computer_vision-msg)


;//! \htmlinclude motor_cmd.msg.html

(cl:defclass <motor_cmd> (roslisp-msg-protocol:ros-message)
  ((left_motor
    :reader left_motor
    :initarg :left_motor
    :type cl:integer
    :initform 0)
   (right_motor
    :reader right_motor
    :initarg :right_motor
    :type cl:integer
    :initform 0))
)

(cl:defclass motor_cmd (<motor_cmd>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <motor_cmd>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'motor_cmd)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name computer_vision-msg:<motor_cmd> is deprecated: use computer_vision-msg:motor_cmd instead.")))

(cl:ensure-generic-function 'left_motor-val :lambda-list '(m))
(cl:defmethod left_motor-val ((m <motor_cmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader computer_vision-msg:left_motor-val is deprecated.  Use computer_vision-msg:left_motor instead.")
  (left_motor m))

(cl:ensure-generic-function 'right_motor-val :lambda-list '(m))
(cl:defmethod right_motor-val ((m <motor_cmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader computer_vision-msg:right_motor-val is deprecated.  Use computer_vision-msg:right_motor instead.")
  (right_motor m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <motor_cmd>) ostream)
  "Serializes a message object of type '<motor_cmd>"
  (cl:let* ((signed (cl:slot-value msg 'left_motor)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'right_motor)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <motor_cmd>) istream)
  "Deserializes a message object of type '<motor_cmd>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'left_motor) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'right_motor) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<motor_cmd>)))
  "Returns string type for a message object of type '<motor_cmd>"
  "computer_vision/motor_cmd")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'motor_cmd)))
  "Returns string type for a message object of type 'motor_cmd"
  "computer_vision/motor_cmd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<motor_cmd>)))
  "Returns md5sum for a message object of type '<motor_cmd>"
  "3a65b38dd0991191156f3432b9d44007")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'motor_cmd)))
  "Returns md5sum for a message object of type 'motor_cmd"
  "3a65b38dd0991191156f3432b9d44007")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<motor_cmd>)))
  "Returns full string definition for message of type '<motor_cmd>"
  (cl:format cl:nil "int32 left_motor~%int32 right_motor~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'motor_cmd)))
  "Returns full string definition for message of type 'motor_cmd"
  (cl:format cl:nil "int32 left_motor~%int32 right_motor~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <motor_cmd>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <motor_cmd>))
  "Converts a ROS message object to a list"
  (cl:list 'motor_cmd
    (cl:cons ':left_motor (left_motor msg))
    (cl:cons ':right_motor (right_motor msg))
))
