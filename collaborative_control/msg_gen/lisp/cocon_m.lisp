; Auto-generated. Do not edit!


(cl:in-package collaborative_control-msg)


;//! \htmlinclude cocon_m.msg.html

(cl:defclass <cocon_m> (roslisp-msg-protocol:ros-message)
  ((robotPos
    :reader robotPos
    :initarg :robotPos
    :type nav_msgs-msg:Odometry
    :initform (cl:make-instance 'nav_msgs-msg:Odometry))
   (obstacles
    :reader obstacles
    :initarg :obstacles
    :type nav_msgs-msg:GridCells
    :initform (cl:make-instance 'nav_msgs-msg:GridCells))
   (target
    :reader target
    :initarg :target
    :type geometry_msgs-msg:Pose2D
    :initform (cl:make-instance 'geometry_msgs-msg:Pose2D))
   (weigthA
    :reader weigthA
    :initarg :weigthA
    :type cl:float
    :initform 0.0)
   (comandA
    :reader comandA
    :initarg :comandA
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (etaA
    :reader etaA
    :initarg :etaA
    :type efficiency-msg:Efficiency_m
    :initform (cl:make-instance 'efficiency-msg:Efficiency_m))
   (weigthB
    :reader weigthB
    :initarg :weigthB
    :type cl:float
    :initform 0.0)
   (comandB
    :reader comandB
    :initarg :comandB
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (etaB
    :reader etaB
    :initarg :etaB
    :type efficiency-msg:Efficiency_m
    :initform (cl:make-instance 'efficiency-msg:Efficiency_m))
   (comandC
    :reader comandC
    :initarg :comandC
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (etaC
    :reader etaC
    :initarg :etaC
    :type efficiency-msg:Efficiency_m
    :initform (cl:make-instance 'efficiency-msg:Efficiency_m)))
)

(cl:defclass cocon_m (<cocon_m>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <cocon_m>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'cocon_m)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name collaborative_control-msg:<cocon_m> is deprecated: use collaborative_control-msg:cocon_m instead.")))

(cl:ensure-generic-function 'robotPos-val :lambda-list '(m))
(cl:defmethod robotPos-val ((m <cocon_m>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader collaborative_control-msg:robotPos-val is deprecated.  Use collaborative_control-msg:robotPos instead.")
  (robotPos m))

(cl:ensure-generic-function 'obstacles-val :lambda-list '(m))
(cl:defmethod obstacles-val ((m <cocon_m>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader collaborative_control-msg:obstacles-val is deprecated.  Use collaborative_control-msg:obstacles instead.")
  (obstacles m))

(cl:ensure-generic-function 'target-val :lambda-list '(m))
(cl:defmethod target-val ((m <cocon_m>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader collaborative_control-msg:target-val is deprecated.  Use collaborative_control-msg:target instead.")
  (target m))

(cl:ensure-generic-function 'weigthA-val :lambda-list '(m))
(cl:defmethod weigthA-val ((m <cocon_m>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader collaborative_control-msg:weigthA-val is deprecated.  Use collaborative_control-msg:weigthA instead.")
  (weigthA m))

(cl:ensure-generic-function 'comandA-val :lambda-list '(m))
(cl:defmethod comandA-val ((m <cocon_m>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader collaborative_control-msg:comandA-val is deprecated.  Use collaborative_control-msg:comandA instead.")
  (comandA m))

(cl:ensure-generic-function 'etaA-val :lambda-list '(m))
(cl:defmethod etaA-val ((m <cocon_m>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader collaborative_control-msg:etaA-val is deprecated.  Use collaborative_control-msg:etaA instead.")
  (etaA m))

(cl:ensure-generic-function 'weigthB-val :lambda-list '(m))
(cl:defmethod weigthB-val ((m <cocon_m>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader collaborative_control-msg:weigthB-val is deprecated.  Use collaborative_control-msg:weigthB instead.")
  (weigthB m))

(cl:ensure-generic-function 'comandB-val :lambda-list '(m))
(cl:defmethod comandB-val ((m <cocon_m>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader collaborative_control-msg:comandB-val is deprecated.  Use collaborative_control-msg:comandB instead.")
  (comandB m))

(cl:ensure-generic-function 'etaB-val :lambda-list '(m))
(cl:defmethod etaB-val ((m <cocon_m>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader collaborative_control-msg:etaB-val is deprecated.  Use collaborative_control-msg:etaB instead.")
  (etaB m))

(cl:ensure-generic-function 'comandC-val :lambda-list '(m))
(cl:defmethod comandC-val ((m <cocon_m>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader collaborative_control-msg:comandC-val is deprecated.  Use collaborative_control-msg:comandC instead.")
  (comandC m))

(cl:ensure-generic-function 'etaC-val :lambda-list '(m))
(cl:defmethod etaC-val ((m <cocon_m>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader collaborative_control-msg:etaC-val is deprecated.  Use collaborative_control-msg:etaC instead.")
  (etaC m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <cocon_m>) ostream)
  "Serializes a message object of type '<cocon_m>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'robotPos) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'obstacles) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'target) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'weigthA))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'comandA) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'etaA) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'weigthB))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'comandB) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'etaB) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'comandC) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'etaC) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <cocon_m>) istream)
  "Deserializes a message object of type '<cocon_m>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'robotPos) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'obstacles) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'target) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'weigthA) (roslisp-utils:decode-single-float-bits bits)))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'comandA) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'etaA) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'weigthB) (roslisp-utils:decode-single-float-bits bits)))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'comandB) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'etaB) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'comandC) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'etaC) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<cocon_m>)))
  "Returns string type for a message object of type '<cocon_m>"
  "collaborative_control/cocon_m")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'cocon_m)))
  "Returns string type for a message object of type 'cocon_m"
  "collaborative_control/cocon_m")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<cocon_m>)))
  "Returns md5sum for a message object of type '<cocon_m>"
  "f5422d830552858705e5ab3bb7b43b70")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'cocon_m)))
  "Returns md5sum for a message object of type 'cocon_m"
  "f5422d830552858705e5ab3bb7b43b70")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<cocon_m>)))
  "Returns full string definition for message of type '<cocon_m>"
  (cl:format cl:nil "nav_msgs/Odometry robotPos~%nav_msgs/GridCells obstacles~%geometry_msgs/Pose2D target~%float32 weigthA~%geometry_msgs/Point comandA~%efficiency/Efficiency_m etaA~%float32 weigthB~%geometry_msgs/Point comandB~%efficiency/Efficiency_m etaB~%geometry_msgs/Point comandC~%efficiency/Efficiency_m etaC~%~%================================================================================~%MSG: nav_msgs/Odometry~%# This represents an estimate of a position and velocity in free space.  ~%# The pose in this message should be specified in the coordinate frame given by header.frame_id.~%# The twist in this message should be specified in the coordinate frame given by the child_frame_id~%Header header~%string child_frame_id~%geometry_msgs/PoseWithCovariance pose~%geometry_msgs/TwistWithCovariance twist~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/PoseWithCovariance~%# This represents a pose in free space with uncertainty.~%~%Pose pose~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/TwistWithCovariance~%# This expresses velocity in free space with uncertianty.~%~%Twist twist~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into it's linear and angular parts. ~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: nav_msgs/GridCells~%#an array of cells in a 2D grid~%Header header~%float32 cell_width~%float32 cell_height~%geometry_msgs/Point[] cells~%~%================================================================================~%MSG: geometry_msgs/Pose2D~%# This expresses a position and orientation on a 2D manifold.~%~%float64 x~%float64 y~%float64 theta~%================================================================================~%MSG: efficiency/Efficiency_m~%float32 Global~%float32 Safety~%float32 Directness~%float32 Smoothness~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'cocon_m)))
  "Returns full string definition for message of type 'cocon_m"
  (cl:format cl:nil "nav_msgs/Odometry robotPos~%nav_msgs/GridCells obstacles~%geometry_msgs/Pose2D target~%float32 weigthA~%geometry_msgs/Point comandA~%efficiency/Efficiency_m etaA~%float32 weigthB~%geometry_msgs/Point comandB~%efficiency/Efficiency_m etaB~%geometry_msgs/Point comandC~%efficiency/Efficiency_m etaC~%~%================================================================================~%MSG: nav_msgs/Odometry~%# This represents an estimate of a position and velocity in free space.  ~%# The pose in this message should be specified in the coordinate frame given by header.frame_id.~%# The twist in this message should be specified in the coordinate frame given by the child_frame_id~%Header header~%string child_frame_id~%geometry_msgs/PoseWithCovariance pose~%geometry_msgs/TwistWithCovariance twist~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/PoseWithCovariance~%# This represents a pose in free space with uncertainty.~%~%Pose pose~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/TwistWithCovariance~%# This expresses velocity in free space with uncertianty.~%~%Twist twist~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into it's linear and angular parts. ~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: nav_msgs/GridCells~%#an array of cells in a 2D grid~%Header header~%float32 cell_width~%float32 cell_height~%geometry_msgs/Point[] cells~%~%================================================================================~%MSG: geometry_msgs/Pose2D~%# This expresses a position and orientation on a 2D manifold.~%~%float64 x~%float64 y~%float64 theta~%================================================================================~%MSG: efficiency/Efficiency_m~%float32 Global~%float32 Safety~%float32 Directness~%float32 Smoothness~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <cocon_m>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'robotPos))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'obstacles))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'target))
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'comandA))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'etaA))
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'comandB))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'etaB))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'comandC))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'etaC))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <cocon_m>))
  "Converts a ROS message object to a list"
  (cl:list 'cocon_m
    (cl:cons ':robotPos (robotPos msg))
    (cl:cons ':obstacles (obstacles msg))
    (cl:cons ':target (target msg))
    (cl:cons ':weigthA (weigthA msg))
    (cl:cons ':comandA (comandA msg))
    (cl:cons ':etaA (etaA msg))
    (cl:cons ':weigthB (weigthB msg))
    (cl:cons ':comandB (comandB msg))
    (cl:cons ':etaB (etaB msg))
    (cl:cons ':comandC (comandC msg))
    (cl:cons ':etaC (etaC msg))
))
