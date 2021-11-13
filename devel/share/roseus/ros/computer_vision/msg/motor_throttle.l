;; Auto-generated. Do not edit!


(when (boundp 'computer_vision::motor_throttle)
  (if (not (find-package "COMPUTER_VISION"))
    (make-package "COMPUTER_VISION"))
  (shadow 'motor_throttle (find-package "COMPUTER_VISION")))
(unless (find-package "COMPUTER_VISION::MOTOR_THROTTLE")
  (make-package "COMPUTER_VISION::MOTOR_THROTTLE"))

(in-package "ROS")
;;//! \htmlinclude motor_throttle.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass computer_vision::motor_throttle
  :super ros::object
  :slots (_header _left_motor _right_motor ))

(defmethod computer_vision::motor_throttle
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:left_motor __left_motor) 0.0)
    ((:right_motor __right_motor) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _left_motor (float __left_motor))
   (setq _right_motor (float __right_motor))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:left_motor
   (&optional __left_motor)
   (if __left_motor (setq _left_motor __left_motor)) _left_motor)
  (:right_motor
   (&optional __right_motor)
   (if __right_motor (setq _right_motor __right_motor)) _right_motor)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float32 _left_motor
    4
    ;; float32 _right_motor
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float32 _left_motor
       (sys::poke _left_motor (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _right_motor
       (sys::poke _right_motor (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float32 _left_motor
     (setq _left_motor (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _right_motor
     (setq _right_motor (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get computer_vision::motor_throttle :md5sum-) "810a90b3c92332e7051784244ee24b0c")
(setf (get computer_vision::motor_throttle :datatype-) "computer_vision/motor_throttle")
(setf (get computer_vision::motor_throttle :definition-)
      "Header header
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

")



(provide :computer_vision/motor_throttle "810a90b3c92332e7051784244ee24b0c")


