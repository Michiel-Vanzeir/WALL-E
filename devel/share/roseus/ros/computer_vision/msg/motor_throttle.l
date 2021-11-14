;; Auto-generated. Do not edit!


(when (boundp 'computer_vision::motor_throttle)
  (if (not (find-package "COMPUTER_VISION"))
    (make-package "COMPUTER_VISION"))
  (shadow 'motor_throttle (find-package "COMPUTER_VISION")))
(unless (find-package "COMPUTER_VISION::MOTOR_THROTTLE")
  (make-package "COMPUTER_VISION::MOTOR_THROTTLE"))

(in-package "ROS")
;;//! \htmlinclude motor_throttle.msg.html


(defclass computer_vision::motor_throttle
  :super ros::object
  :slots (_left_motor _right_motor ))

(defmethod computer_vision::motor_throttle
  (:init
   (&key
    ((:left_motor __left_motor) 0.0)
    ((:right_motor __right_motor) 0.0)
    )
   (send-super :init)
   (setq _left_motor (float __left_motor))
   (setq _right_motor (float __right_motor))
   self)
  (:left_motor
   (&optional __left_motor)
   (if __left_motor (setq _left_motor __left_motor)) _left_motor)
  (:right_motor
   (&optional __right_motor)
   (if __right_motor (setq _right_motor __right_motor)) _right_motor)
  (:serialization-length
   ()
   (+
    ;; float32 _left_motor
    4
    ;; float32 _right_motor
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _left_motor
       (sys::poke _left_motor (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _right_motor
       (sys::poke _right_motor (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _left_motor
     (setq _left_motor (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _right_motor
     (setq _right_motor (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get computer_vision::motor_throttle :md5sum-) "3e3717ac8e9443aa62d7102a5860f5e7")
(setf (get computer_vision::motor_throttle :datatype-) "computer_vision/motor_throttle")
(setf (get computer_vision::motor_throttle :definition-)
      "float32 left_motor
float32 right_motor

")



(provide :computer_vision/motor_throttle "3e3717ac8e9443aa62d7102a5860f5e7")


