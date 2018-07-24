; Auto-generated. Do not edit!


(cl:in-package kuka_arm-srv)


;//! \htmlinclude CalculateIK-request.msg.html

(cl:defclass <CalculateIK-request> (roslisp-msg-protocol:ros-message)
  ((poses
    :reader poses
    :initarg :poses
    :type (cl:vector geometry_msgs-msg:Pose)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Pose :initial-element (cl:make-instance 'geometry_msgs-msg:Pose))))
)

(cl:defclass CalculateIK-request (<CalculateIK-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CalculateIK-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CalculateIK-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kuka_arm-srv:<CalculateIK-request> is deprecated: use kuka_arm-srv:CalculateIK-request instead.")))

(cl:ensure-generic-function 'poses-val :lambda-list '(m))
(cl:defmethod poses-val ((m <CalculateIK-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kuka_arm-srv:poses-val is deprecated.  Use kuka_arm-srv:poses instead.")
  (poses m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CalculateIK-request>) ostream)
  "Serializes a message object of type '<CalculateIK-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'poses))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'poses))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CalculateIK-request>) istream)
  "Deserializes a message object of type '<CalculateIK-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'poses) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'poses)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Pose))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CalculateIK-request>)))
  "Returns string type for a service object of type '<CalculateIK-request>"
  "kuka_arm/CalculateIKRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CalculateIK-request)))
  "Returns string type for a service object of type 'CalculateIK-request"
  "kuka_arm/CalculateIKRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CalculateIK-request>)))
  "Returns md5sum for a message object of type '<CalculateIK-request>"
  "e2841ca7335735bd34d77773a974ca4b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CalculateIK-request)))
  "Returns md5sum for a message object of type 'CalculateIK-request"
  "e2841ca7335735bd34d77773a974ca4b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CalculateIK-request>)))
  "Returns full string definition for message of type '<CalculateIK-request>"
  (cl:format cl:nil "geometry_msgs/Pose[] poses~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CalculateIK-request)))
  "Returns full string definition for message of type 'CalculateIK-request"
  (cl:format cl:nil "geometry_msgs/Pose[] poses~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CalculateIK-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'poses) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CalculateIK-request>))
  "Converts a ROS message object to a list"
  (cl:list 'CalculateIK-request
    (cl:cons ':poses (poses msg))
))
;//! \htmlinclude CalculateIK-response.msg.html

(cl:defclass <CalculateIK-response> (roslisp-msg-protocol:ros-message)
  ((points
    :reader points
    :initarg :points
    :type (cl:vector trajectory_msgs-msg:JointTrajectoryPoint)
   :initform (cl:make-array 0 :element-type 'trajectory_msgs-msg:JointTrajectoryPoint :initial-element (cl:make-instance 'trajectory_msgs-msg:JointTrajectoryPoint))))
)

(cl:defclass CalculateIK-response (<CalculateIK-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CalculateIK-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CalculateIK-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kuka_arm-srv:<CalculateIK-response> is deprecated: use kuka_arm-srv:CalculateIK-response instead.")))

(cl:ensure-generic-function 'points-val :lambda-list '(m))
(cl:defmethod points-val ((m <CalculateIK-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kuka_arm-srv:points-val is deprecated.  Use kuka_arm-srv:points instead.")
  (points m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CalculateIK-response>) ostream)
  "Serializes a message object of type '<CalculateIK-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'points))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'points))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CalculateIK-response>) istream)
  "Deserializes a message object of type '<CalculateIK-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'points) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'points)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'trajectory_msgs-msg:JointTrajectoryPoint))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CalculateIK-response>)))
  "Returns string type for a service object of type '<CalculateIK-response>"
  "kuka_arm/CalculateIKResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CalculateIK-response)))
  "Returns string type for a service object of type 'CalculateIK-response"
  "kuka_arm/CalculateIKResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CalculateIK-response>)))
  "Returns md5sum for a message object of type '<CalculateIK-response>"
  "e2841ca7335735bd34d77773a974ca4b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CalculateIK-response)))
  "Returns md5sum for a message object of type 'CalculateIK-response"
  "e2841ca7335735bd34d77773a974ca4b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CalculateIK-response>)))
  "Returns full string definition for message of type '<CalculateIK-response>"
  (cl:format cl:nil "trajectory_msgs/JointTrajectoryPoint[] points~%~%~%================================================================================~%MSG: trajectory_msgs/JointTrajectoryPoint~%# Each trajectory point specifies either positions[, velocities[, accelerations]]~%# or positions[, effort] for the trajectory to be executed.~%# All specified values are in the same order as the joint names in JointTrajectory.msg~%~%float64[] positions~%float64[] velocities~%float64[] accelerations~%float64[] effort~%duration time_from_start~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CalculateIK-response)))
  "Returns full string definition for message of type 'CalculateIK-response"
  (cl:format cl:nil "trajectory_msgs/JointTrajectoryPoint[] points~%~%~%================================================================================~%MSG: trajectory_msgs/JointTrajectoryPoint~%# Each trajectory point specifies either positions[, velocities[, accelerations]]~%# or positions[, effort] for the trajectory to be executed.~%# All specified values are in the same order as the joint names in JointTrajectory.msg~%~%float64[] positions~%float64[] velocities~%float64[] accelerations~%float64[] effort~%duration time_from_start~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CalculateIK-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'points) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CalculateIK-response>))
  "Converts a ROS message object to a list"
  (cl:list 'CalculateIK-response
    (cl:cons ':points (points msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'CalculateIK)))
  'CalculateIK-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'CalculateIK)))
  'CalculateIK-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CalculateIK)))
  "Returns string type for a service object of type '<CalculateIK>"
  "kuka_arm/CalculateIK")