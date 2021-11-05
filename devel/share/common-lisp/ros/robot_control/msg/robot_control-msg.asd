
(cl:in-package :asdf)

(defsystem "robot_control-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "motor_cmd" :depends-on ("_package_motor_cmd"))
    (:file "_package_motor_cmd" :depends-on ("_package"))
  ))