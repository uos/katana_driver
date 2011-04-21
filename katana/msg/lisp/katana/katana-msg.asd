
(in-package :asdf)

(defsystem "katana-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :actionlib_msgs-msg
 :roslib-msg
 :sensor_msgs-msg
)
  :components ((:file "_package")
    (:file "JointMovementGoal" :depends-on ("_package"))
    (:file "_package_JointMovementGoal" :depends-on ("_package"))
    (:file "JointMovementAction" :depends-on ("_package"))
    (:file "_package_JointMovementAction" :depends-on ("_package"))
    (:file "JointMovementResult" :depends-on ("_package"))
    (:file "_package_JointMovementResult" :depends-on ("_package"))
    (:file "JointMovementActionGoal" :depends-on ("_package"))
    (:file "_package_JointMovementActionGoal" :depends-on ("_package"))
    (:file "JointMovementFeedback" :depends-on ("_package"))
    (:file "_package_JointMovementFeedback" :depends-on ("_package"))
    (:file "JointMovementActionResult" :depends-on ("_package"))
    (:file "_package_JointMovementActionResult" :depends-on ("_package"))
    (:file "JointMovementActionFeedback" :depends-on ("_package"))
    (:file "_package_JointMovementActionFeedback" :depends-on ("_package"))
    ))
