;; Auto-generated. Do not edit!


(when (boundp 'robot_control::motor_cmd)
  (if (not (find-package "ROBOT_CONTROL"))
    (make-package "ROBOT_CONTROL"))
  (shadow 'motor_cmd (find-package "ROBOT_CONTROL")))
(unless (find-package "ROBOT_CONTROL::MOTOR_CMD")
  (make-package "ROBOT_CONTROL::MOTOR_CMD"))

(in-package "ROS")
;;//! \htmlinclude motor_cmd.msg.html


(defclass robot_control::motor_cmd
  :super ros::object
  :slots (_left_motor _right_motor ))

(defmethod robot_control::motor_cmd
  (:init
   (&key
    ((:left_motor __left_motor) 0)
    ((:right_motor __right_motor) 0)
    )
   (send-super :init)
   (setq _left_motor (round __left_motor))
   (setq _right_motor (round __right_motor))
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
    ;; int32 _left_motor
    4
    ;; int32 _right_motor
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _left_motor
       (write-long _left_motor s)
     ;; int32 _right_motor
       (write-long _right_motor s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _left_motor
     (setq _left_motor (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _right_motor
     (setq _right_motor (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get robot_control::motor_cmd :md5sum-) "3a65b38dd0991191156f3432b9d44007")
(setf (get robot_control::motor_cmd :datatype-) "robot_control/motor_cmd")
(setf (get robot_control::motor_cmd :definition-)
      "int32 left_motor
int32 right_motor

")



(provide :robot_control/motor_cmd "3a65b38dd0991191156f3432b9d44007")


