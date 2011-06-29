; Auto-generated. Do not edit!


(cl:in-package katana_interpolated_ik_motion_planner-srv)


;//! \htmlinclude SetInterpolatedIKMotionPlanParams-request.msg.html

(cl:defclass <SetInterpolatedIKMotionPlanParams-request> (roslisp-msg-protocol:ros-message)
  ((num_steps
    :reader num_steps
    :initarg :num_steps
    :type cl:integer
    :initform 0)
   (consistent_angle
    :reader consistent_angle
    :initarg :consistent_angle
    :type cl:float
    :initform 0.0)
   (collision_check_resolution
    :reader collision_check_resolution
    :initarg :collision_check_resolution
    :type cl:integer
    :initform 0)
   (steps_before_abort
    :reader steps_before_abort
    :initarg :steps_before_abort
    :type cl:integer
    :initform 0)
   (pos_spacing
    :reader pos_spacing
    :initarg :pos_spacing
    :type cl:float
    :initform 0.0)
   (rot_spacing
    :reader rot_spacing
    :initarg :rot_spacing
    :type cl:float
    :initform 0.0)
   (collision_aware
    :reader collision_aware
    :initarg :collision_aware
    :type cl:integer
    :initform 0)
   (start_from_end
    :reader start_from_end
    :initarg :start_from_end
    :type cl:integer
    :initform 0)
   (max_joint_vels
    :reader max_joint_vels
    :initarg :max_joint_vels
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (max_joint_accs
    :reader max_joint_accs
    :initarg :max_joint_accs
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass SetInterpolatedIKMotionPlanParams-request (<SetInterpolatedIKMotionPlanParams-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetInterpolatedIKMotionPlanParams-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetInterpolatedIKMotionPlanParams-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name katana_interpolated_ik_motion_planner-srv:<SetInterpolatedIKMotionPlanParams-request> is deprecated: use katana_interpolated_ik_motion_planner-srv:SetInterpolatedIKMotionPlanParams-request instead.")))

(cl:ensure-generic-function 'num_steps-val :lambda-list '(m))
(cl:defmethod num_steps-val ((m <SetInterpolatedIKMotionPlanParams-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader katana_interpolated_ik_motion_planner-srv:num_steps-val is deprecated.  Use katana_interpolated_ik_motion_planner-srv:num_steps instead.")
  (num_steps m))

(cl:ensure-generic-function 'consistent_angle-val :lambda-list '(m))
(cl:defmethod consistent_angle-val ((m <SetInterpolatedIKMotionPlanParams-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader katana_interpolated_ik_motion_planner-srv:consistent_angle-val is deprecated.  Use katana_interpolated_ik_motion_planner-srv:consistent_angle instead.")
  (consistent_angle m))

(cl:ensure-generic-function 'collision_check_resolution-val :lambda-list '(m))
(cl:defmethod collision_check_resolution-val ((m <SetInterpolatedIKMotionPlanParams-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader katana_interpolated_ik_motion_planner-srv:collision_check_resolution-val is deprecated.  Use katana_interpolated_ik_motion_planner-srv:collision_check_resolution instead.")
  (collision_check_resolution m))

(cl:ensure-generic-function 'steps_before_abort-val :lambda-list '(m))
(cl:defmethod steps_before_abort-val ((m <SetInterpolatedIKMotionPlanParams-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader katana_interpolated_ik_motion_planner-srv:steps_before_abort-val is deprecated.  Use katana_interpolated_ik_motion_planner-srv:steps_before_abort instead.")
  (steps_before_abort m))

(cl:ensure-generic-function 'pos_spacing-val :lambda-list '(m))
(cl:defmethod pos_spacing-val ((m <SetInterpolatedIKMotionPlanParams-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader katana_interpolated_ik_motion_planner-srv:pos_spacing-val is deprecated.  Use katana_interpolated_ik_motion_planner-srv:pos_spacing instead.")
  (pos_spacing m))

(cl:ensure-generic-function 'rot_spacing-val :lambda-list '(m))
(cl:defmethod rot_spacing-val ((m <SetInterpolatedIKMotionPlanParams-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader katana_interpolated_ik_motion_planner-srv:rot_spacing-val is deprecated.  Use katana_interpolated_ik_motion_planner-srv:rot_spacing instead.")
  (rot_spacing m))

(cl:ensure-generic-function 'collision_aware-val :lambda-list '(m))
(cl:defmethod collision_aware-val ((m <SetInterpolatedIKMotionPlanParams-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader katana_interpolated_ik_motion_planner-srv:collision_aware-val is deprecated.  Use katana_interpolated_ik_motion_planner-srv:collision_aware instead.")
  (collision_aware m))

(cl:ensure-generic-function 'start_from_end-val :lambda-list '(m))
(cl:defmethod start_from_end-val ((m <SetInterpolatedIKMotionPlanParams-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader katana_interpolated_ik_motion_planner-srv:start_from_end-val is deprecated.  Use katana_interpolated_ik_motion_planner-srv:start_from_end instead.")
  (start_from_end m))

(cl:ensure-generic-function 'max_joint_vels-val :lambda-list '(m))
(cl:defmethod max_joint_vels-val ((m <SetInterpolatedIKMotionPlanParams-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader katana_interpolated_ik_motion_planner-srv:max_joint_vels-val is deprecated.  Use katana_interpolated_ik_motion_planner-srv:max_joint_vels instead.")
  (max_joint_vels m))

(cl:ensure-generic-function 'max_joint_accs-val :lambda-list '(m))
(cl:defmethod max_joint_accs-val ((m <SetInterpolatedIKMotionPlanParams-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader katana_interpolated_ik_motion_planner-srv:max_joint_accs-val is deprecated.  Use katana_interpolated_ik_motion_planner-srv:max_joint_accs instead.")
  (max_joint_accs m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetInterpolatedIKMotionPlanParams-request>) ostream)
  "Serializes a message object of type '<SetInterpolatedIKMotionPlanParams-request>"
  (cl:let* ((signed (cl:slot-value msg 'num_steps)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'consistent_angle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'collision_check_resolution)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'steps_before_abort)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'pos_spacing))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'rot_spacing))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'collision_aware)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'start_from_end)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'max_joint_vels))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'max_joint_vels))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'max_joint_accs))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'max_joint_accs))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetInterpolatedIKMotionPlanParams-request>) istream)
  "Deserializes a message object of type '<SetInterpolatedIKMotionPlanParams-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'num_steps) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'consistent_angle) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'collision_check_resolution) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'steps_before_abort) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pos_spacing) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'rot_spacing) (roslisp-utils:decode-double-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'collision_aware)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'start_from_end)) (cl:read-byte istream))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'max_joint_vels) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'max_joint_vels)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'max_joint_accs) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'max_joint_accs)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetInterpolatedIKMotionPlanParams-request>)))
  "Returns string type for a service object of type '<SetInterpolatedIKMotionPlanParams-request>"
  "katana_interpolated_ik_motion_planner/SetInterpolatedIKMotionPlanParamsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetInterpolatedIKMotionPlanParams-request)))
  "Returns string type for a service object of type 'SetInterpolatedIKMotionPlanParams-request"
  "katana_interpolated_ik_motion_planner/SetInterpolatedIKMotionPlanParamsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetInterpolatedIKMotionPlanParams-request>)))
  "Returns md5sum for a message object of type '<SetInterpolatedIKMotionPlanParams-request>"
  "351122754b3043b9f5602d68d4eec5db")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetInterpolatedIKMotionPlanParams-request)))
  "Returns md5sum for a message object of type 'SetInterpolatedIKMotionPlanParams-request"
  "351122754b3043b9f5602d68d4eec5db")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetInterpolatedIKMotionPlanParams-request>)))
  "Returns full string definition for message of type '<SetInterpolatedIKMotionPlanParams-request>"
  (cl:format cl:nil "~%~%~%int32 num_steps~%~%~%~%float64 consistent_angle~%~%~%~%int32 collision_check_resolution~%~%~%~%~%~%int32 steps_before_abort~%~%~%~%float64 pos_spacing~%~%~%~%float64 rot_spacing~%~%~%~%byte collision_aware~%~%~%~%byte start_from_end~%~%~%~%float64[] max_joint_vels~%~%~%~%float64[] max_joint_accs~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetInterpolatedIKMotionPlanParams-request)))
  "Returns full string definition for message of type 'SetInterpolatedIKMotionPlanParams-request"
  (cl:format cl:nil "~%~%~%int32 num_steps~%~%~%~%float64 consistent_angle~%~%~%~%int32 collision_check_resolution~%~%~%~%~%~%int32 steps_before_abort~%~%~%~%float64 pos_spacing~%~%~%~%float64 rot_spacing~%~%~%~%byte collision_aware~%~%~%~%byte start_from_end~%~%~%~%float64[] max_joint_vels~%~%~%~%float64[] max_joint_accs~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetInterpolatedIKMotionPlanParams-request>))
  (cl:+ 0
     4
     8
     4
     4
     8
     8
     1
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'max_joint_vels) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'max_joint_accs) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetInterpolatedIKMotionPlanParams-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetInterpolatedIKMotionPlanParams-request
    (cl:cons ':num_steps (num_steps msg))
    (cl:cons ':consistent_angle (consistent_angle msg))
    (cl:cons ':collision_check_resolution (collision_check_resolution msg))
    (cl:cons ':steps_before_abort (steps_before_abort msg))
    (cl:cons ':pos_spacing (pos_spacing msg))
    (cl:cons ':rot_spacing (rot_spacing msg))
    (cl:cons ':collision_aware (collision_aware msg))
    (cl:cons ':start_from_end (start_from_end msg))
    (cl:cons ':max_joint_vels (max_joint_vels msg))
    (cl:cons ':max_joint_accs (max_joint_accs msg))
))
;//! \htmlinclude SetInterpolatedIKMotionPlanParams-response.msg.html

(cl:defclass <SetInterpolatedIKMotionPlanParams-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass SetInterpolatedIKMotionPlanParams-response (<SetInterpolatedIKMotionPlanParams-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetInterpolatedIKMotionPlanParams-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetInterpolatedIKMotionPlanParams-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name katana_interpolated_ik_motion_planner-srv:<SetInterpolatedIKMotionPlanParams-response> is deprecated: use katana_interpolated_ik_motion_planner-srv:SetInterpolatedIKMotionPlanParams-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetInterpolatedIKMotionPlanParams-response>) ostream)
  "Serializes a message object of type '<SetInterpolatedIKMotionPlanParams-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetInterpolatedIKMotionPlanParams-response>) istream)
  "Deserializes a message object of type '<SetInterpolatedIKMotionPlanParams-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetInterpolatedIKMotionPlanParams-response>)))
  "Returns string type for a service object of type '<SetInterpolatedIKMotionPlanParams-response>"
  "katana_interpolated_ik_motion_planner/SetInterpolatedIKMotionPlanParamsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetInterpolatedIKMotionPlanParams-response)))
  "Returns string type for a service object of type 'SetInterpolatedIKMotionPlanParams-response"
  "katana_interpolated_ik_motion_planner/SetInterpolatedIKMotionPlanParamsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetInterpolatedIKMotionPlanParams-response>)))
  "Returns md5sum for a message object of type '<SetInterpolatedIKMotionPlanParams-response>"
  "351122754b3043b9f5602d68d4eec5db")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetInterpolatedIKMotionPlanParams-response)))
  "Returns md5sum for a message object of type 'SetInterpolatedIKMotionPlanParams-response"
  "351122754b3043b9f5602d68d4eec5db")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetInterpolatedIKMotionPlanParams-response>)))
  "Returns full string definition for message of type '<SetInterpolatedIKMotionPlanParams-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetInterpolatedIKMotionPlanParams-response)))
  "Returns full string definition for message of type 'SetInterpolatedIKMotionPlanParams-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetInterpolatedIKMotionPlanParams-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetInterpolatedIKMotionPlanParams-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetInterpolatedIKMotionPlanParams-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetInterpolatedIKMotionPlanParams)))
  'SetInterpolatedIKMotionPlanParams-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetInterpolatedIKMotionPlanParams)))
  'SetInterpolatedIKMotionPlanParams-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetInterpolatedIKMotionPlanParams)))
  "Returns string type for a service object of type '<SetInterpolatedIKMotionPlanParams>"
  "katana_interpolated_ik_motion_planner/SetInterpolatedIKMotionPlanParams")
