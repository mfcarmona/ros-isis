; Auto-generated. Do not edit!


(cl:in-package efficiency-srv)


;//! \htmlinclude Efficiency_s2-request.msg.html

(cl:defclass <Efficiency_s2-request> (roslisp-msg-protocol:ros-message)
  ((comand
    :reader comand
    :initarg :comand
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (robotPos
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
    :initform (cl:make-instance 'geometry_msgs-msg:Pose2D)))
)

(cl:defclass Efficiency_s2-request (<Efficiency_s2-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Efficiency_s2-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Efficiency_s2-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name efficiency-srv:<Efficiency_s2-request> is deprecated: use efficiency-srv:Efficiency_s2-request instead.")))

(cl:ensure-generic-function 'comand-val :lambda-list '(m))
(cl:defmethod comand-val ((m <Efficiency_s2-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader efficiency-srv:comand-val is deprecated.  Use efficiency-srv:comand instead.")
  (comand m))

(cl:ensure-generic-function 'robotPos-val :lambda-list '(m))
(cl:defmethod robotPos-val ((m <Efficiency_s2-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader efficiency-srv:robotPos-val is deprecated.  Use efficiency-srv:robotPos instead.")
  (robotPos m))

(cl:ensure-generic-function 'obstacles-val :lambda-list '(m))
(cl:defmethod obstacles-val ((m <Efficiency_s2-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader efficiency-srv:obstacles-val is deprecated.  Use efficiency-srv:obstacles instead.")
  (obstacles m))

(cl:ensure-generic-function 'target-val :lambda-list '(m))
(cl:defmethod target-val ((m <Efficiency_s2-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader efficiency-srv:target-val is deprecated.  Use efficiency-srv:target instead.")
  (target m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Efficiency_s2-request>) ostream)
  "Serializes a message object of type '<Efficiency_s2-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'comand) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'robotPos) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'obstacles) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'target) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Efficiency_s2-request>) istream)
  "Deserializes a message object of type '<Efficiency_s2-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'comand) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'robotPos) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'obstacles) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'target) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Efficiency_s2-request>)))
  "Returns string type for a service object of type '<Efficiency_s2-request>"
  "efficiency/Efficiency_s2Request")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Efficiency_s2-request)))
  "Returns string type for a service object of type 'Efficiency_s2-request"
  "efficiency/Efficiency_s2Request")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Efficiency_s2-request>)))
  "Returns md5sum for a message object of type '<Efficiency_s2-request>"
  "803581245f4fa5ea991fd5cd635af433")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Efficiency_s2-request)))
  "Returns md5sum for a message object of type 'Efficiency_s2-request"
  "803581245f4fa5ea991fd5cd635af433")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Efficiency_s2-request>)))
  "Returns full string definition for message of type '<Efficiency_s2-request>"
  (cl:format cl:nil "geometry_msgs/Point comand~%nav_msgs/Odometry robotPos~%nav_msgs/GridCells obstacles~%geometry_msgs/Pose2D target~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: nav_msgs/Odometry~%# This represents an estimate of a position and velocity in free space.  ~%# The pose in this message should be specified in the coordinate frame given by header.frame_id.~%# The twist in this message should be specified in the coordinate frame given by the child_frame_id~%Header header~%string child_frame_id~%geometry_msgs/PoseWithCovariance pose~%geometry_msgs/TwistWithCovariance twist~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/PoseWithCovariance~%# This represents a pose in free space with uncertainty.~%~%Pose pose~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/TwistWithCovariance~%# This expresses velocity in free space with uncertianty.~%~%Twist twist~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into it's linear and angular parts. ~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: nav_msgs/GridCells~%#an array of cells in a 2D grid~%Header header~%float32 cell_width~%float32 cell_height~%geometry_msgs/Point[] cells~%~%================================================================================~%MSG: geometry_msgs/Pose2D~%# This expresses a position and orientation on a 2D manifold.~%~%float64 x~%float64 y~%float64 theta~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Efficiency_s2-request)))
  "Returns full string definition for message of type 'Efficiency_s2-request"
  (cl:format cl:nil "geometry_msgs/Point comand~%nav_msgs/Odometry robotPos~%nav_msgs/GridCells obstacles~%geometry_msgs/Pose2D target~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: nav_msgs/Odometry~%# This represents an estimate of a position and velocity in free space.  ~%# The pose in this message should be specified in the coordinate frame given by header.frame_id.~%# The twist in this message should be specified in the coordinate frame given by the child_frame_id~%Header header~%string child_frame_id~%geometry_msgs/PoseWithCovariance pose~%geometry_msgs/TwistWithCovariance twist~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/PoseWithCovariance~%# This represents a pose in free space with uncertainty.~%~%Pose pose~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/TwistWithCovariance~%# This expresses velocity in free space with uncertianty.~%~%Twist twist~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into it's linear and angular parts. ~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: nav_msgs/GridCells~%#an array of cells in a 2D grid~%Header header~%float32 cell_width~%float32 cell_height~%geometry_msgs/Point[] cells~%~%================================================================================~%MSG: geometry_msgs/Pose2D~%# This expresses a position and orientation on a 2D manifold.~%~%float64 x~%float64 y~%float64 theta~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Efficiency_s2-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'comand))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'robotPos))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'obstacles))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'target))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Efficiency_s2-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Efficiency_s2-request
    (cl:cons ':comand (comand msg))
    (cl:cons ':robotPos (robotPos msg))
    (cl:cons ':obstacles (obstacles msg))
    (cl:cons ':target (target msg))
))
;//! \htmlinclude Efficiency_s2-response.msg.html

(cl:defclass <Efficiency_s2-response> (roslisp-msg-protocol:ros-message)
  ((eta
    :reader eta
    :initarg :eta
    :type efficiency-msg:Efficiency_m
    :initform (cl:make-instance 'efficiency-msg:Efficiency_m)))
)

(cl:defclass Efficiency_s2-response (<Efficiency_s2-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Efficiency_s2-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Efficiency_s2-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name efficiency-srv:<Efficiency_s2-response> is deprecated: use efficiency-srv:Efficiency_s2-response instead.")))

(cl:ensure-generic-function 'eta-val :lambda-list '(m))
(cl:defmethod eta-val ((m <Efficiency_s2-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader efficiency-srv:eta-val is deprecated.  Use efficiency-srv:eta instead.")
  (eta m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Efficiency_s2-response>) ostream)
  "Serializes a message object of type '<Efficiency_s2-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'eta) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Efficiency_s2-response>) istream)
  "Deserializes a message object of type '<Efficiency_s2-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'eta) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Efficiency_s2-response>)))
  "Returns string type for a service object of type '<Efficiency_s2-response>"
  "efficiency/Efficiency_s2Response")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Efficiency_s2-response)))
  "Returns string type for a service object of type 'Efficiency_s2-response"
  "efficiency/Efficiency_s2Response")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Efficiency_s2-response>)))
  "Returns md5sum for a message object of type '<Efficiency_s2-response>"
  "803581245f4fa5ea991fd5cd635af433")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Efficiency_s2-response)))
  "Returns md5sum for a message object of type 'Efficiency_s2-response"
  "803581245f4fa5ea991fd5cd635af433")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Efficiency_s2-response>)))
  "Returns full string definition for message of type '<Efficiency_s2-response>"
  (cl:format cl:nil "Efficiency_m eta~%~%~%~%================================================================================~%MSG: efficiency/Efficiency_m~%float32 Global~%float32 Safety~%float32 Directness~%float32 Smoothness~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Efficiency_s2-response)))
  "Returns full string definition for message of type 'Efficiency_s2-response"
  (cl:format cl:nil "Efficiency_m eta~%~%~%~%================================================================================~%MSG: efficiency/Efficiency_m~%float32 Global~%float32 Safety~%float32 Directness~%float32 Smoothness~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Efficiency_s2-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'eta))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Efficiency_s2-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Efficiency_s2-response
    (cl:cons ':eta (eta msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Efficiency_s2)))
  'Efficiency_s2-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Efficiency_s2)))
  'Efficiency_s2-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Efficiency_s2)))
  "Returns string type for a service object of type '<Efficiency_s2>"
  "efficiency/Efficiency_s2")
