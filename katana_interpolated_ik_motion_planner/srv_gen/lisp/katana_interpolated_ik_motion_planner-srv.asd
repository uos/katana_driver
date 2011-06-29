
(cl:in-package :asdf)

(defsystem "katana_interpolated_ik_motion_planner-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "SetInterpolatedIKMotionPlanParams" :depends-on ("_package_SetInterpolatedIKMotionPlanParams"))
    (:file "_package_SetInterpolatedIKMotionPlanParams" :depends-on ("_package"))
  ))