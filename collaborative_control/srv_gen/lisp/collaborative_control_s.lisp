; Auto-generated. Do not edit!


(cl:in-package collaborative_control-srv)


;//! \htmlinclude collaborative_control_s-request.msg.html

(cl:defclass <collaborative_control_s-request> (roslisp-msg-protocol:ros-message)
  ((weigthA
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
    :initform (cl:make-instance 'efficiency-msg:Efficiency_m)))
)

(cl:defclass collaborative_control_s-request (<collaborative_control_s-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <collaborative_control_s-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'collaborative_control_s-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name collaborative_control-srv:<collaborative_control_s-request> is deprecated: use collaborative_control-srv:collaborative_control_s-request instead.")))

(cl:ensure-generic-function 'weigthA-val :lambda-list '(m))
(cl:defmethod weigthA-val ((m <collaborative_control_s-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader collaborative_control-srv:weigthA-val is deprecated.  Use collaborative_control-srv:weigthA instead.")
  (weigthA m))

(cl:ensure-generic-function 'comandA-val :lambda-list '(m))
(cl:defmethod comandA-val ((m <collaborative_control_s-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader collaborative_control-srv:comandA-val is deprecated.  Use collaborative_control-srv:comandA instead.")
  (comandA m))

(cl:ensure-generic-function 'etaA-val :lambda-list '(m))
(cl:defmethod etaA-val ((m <collaborative_control_s-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader collaborative_control-srv:etaA-val is deprecated.  Use collaborative_control-srv:etaA instead.")
  (etaA m))

(cl:ensure-generic-function 'weigthB-val :lambda-list '(m))
(cl:defmethod weigthB-val ((m <collaborative_control_s-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader collaborative_control-srv:weigthB-val is deprecated.  Use collaborative_control-srv:weigthB instead.")
  (weigthB m))

(cl:ensure-generic-function 'comandB-val :lambda-list '(m))
(cl:defmethod comandB-val ((m <collaborative_control_s-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader collaborative_control-srv:comandB-val is deprecated.  Use collaborative_control-srv:comandB instead.")
  (comandB m))

(cl:ensure-generic-function 'etaB-val :lambda-list '(m))
(cl:defmethod etaB-val ((m <collaborative_control_s-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader collaborative_control-srv:etaB-val is deprecated.  Use collaborative_control-srv:etaB instead.")
  (etaB m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <collaborative_control_s-request>) ostream)
  "Serializes a message object of type '<collaborative_control_s-request>"
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
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <collaborative_control_s-request>) istream)
  "Deserializes a message object of type '<collaborative_control_s-request>"
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
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<collaborative_control_s-request>)))
  "Returns string type for a service object of type '<collaborative_control_s-request>"
  "collaborative_control/collaborative_control_sRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'collaborative_control_s-request)))
  "Returns string type for a service object of type 'collaborative_control_s-request"
  "collaborative_control/collaborative_control_sRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<collaborative_control_s-request>)))
  "Returns md5sum for a message object of type '<collaborative_control_s-request>"
  "6f29b80ef1ae1821153e3c4550e43179")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'collaborative_control_s-request)))
  "Returns md5sum for a message object of type 'collaborative_control_s-request"
  "6f29b80ef1ae1821153e3c4550e43179")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<collaborative_control_s-request>)))
  "Returns full string definition for message of type '<collaborative_control_s-request>"
  (cl:format cl:nil "float32 weigthA~%geometry_msgs/Point comandA~%efficiency/Efficiency_m etaA~%float32 weigthB~%geometry_msgs/Point comandB~%efficiency/Efficiency_m etaB~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: efficiency/Efficiency_m~%float32 Global~%float32 Safety~%float32 Directness~%float32 Smoothness~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'collaborative_control_s-request)))
  "Returns full string definition for message of type 'collaborative_control_s-request"
  (cl:format cl:nil "float32 weigthA~%geometry_msgs/Point comandA~%efficiency/Efficiency_m etaA~%float32 weigthB~%geometry_msgs/Point comandB~%efficiency/Efficiency_m etaB~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: efficiency/Efficiency_m~%float32 Global~%float32 Safety~%float32 Directness~%float32 Smoothness~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <collaborative_control_s-request>))
  (cl:+ 0
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'comandA))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'etaA))
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'comandB))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'etaB))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <collaborative_control_s-request>))
  "Converts a ROS message object to a list"
  (cl:list 'collaborative_control_s-request
    (cl:cons ':weigthA (weigthA msg))
    (cl:cons ':comandA (comandA msg))
    (cl:cons ':etaA (etaA msg))
    (cl:cons ':weigthB (weigthB msg))
    (cl:cons ':comandB (comandB msg))
    (cl:cons ':etaB (etaB msg))
))
;//! \htmlinclude collaborative_control_s-response.msg.html

(cl:defclass <collaborative_control_s-response> (roslisp-msg-protocol:ros-message)
  ((comandC
    :reader comandC
    :initarg :comandC
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point)))
)

(cl:defclass collaborative_control_s-response (<collaborative_control_s-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <collaborative_control_s-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'collaborative_control_s-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name collaborative_control-srv:<collaborative_control_s-response> is deprecated: use collaborative_control-srv:collaborative_control_s-response instead.")))

(cl:ensure-generic-function 'comandC-val :lambda-list '(m))
(cl:defmethod comandC-val ((m <collaborative_control_s-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader collaborative_control-srv:comandC-val is deprecated.  Use collaborative_control-srv:comandC instead.")
  (comandC m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <collaborative_control_s-response>) ostream)
  "Serializes a message object of type '<collaborative_control_s-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'comandC) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <collaborative_control_s-response>) istream)
  "Deserializes a message object of type '<collaborative_control_s-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'comandC) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<collaborative_control_s-response>)))
  "Returns string type for a service object of type '<collaborative_control_s-response>"
  "collaborative_control/collaborative_control_sResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'collaborative_control_s-response)))
  "Returns string type for a service object of type 'collaborative_control_s-response"
  "collaborative_control/collaborative_control_sResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<collaborative_control_s-response>)))
  "Returns md5sum for a message object of type '<collaborative_control_s-response>"
  "6f29b80ef1ae1821153e3c4550e43179")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'collaborative_control_s-response)))
  "Returns md5sum for a message object of type 'collaborative_control_s-response"
  "6f29b80ef1ae1821153e3c4550e43179")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<collaborative_control_s-response>)))
  "Returns full string definition for message of type '<collaborative_control_s-response>"
  (cl:format cl:nil "geometry_msgs/Point comandC~%~%~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'collaborative_control_s-response)))
  "Returns full string definition for message of type 'collaborative_control_s-response"
  (cl:format cl:nil "geometry_msgs/Point comandC~%~%~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <collaborative_control_s-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'comandC))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <collaborative_control_s-response>))
  "Converts a ROS message object to a list"
  (cl:list 'collaborative_control_s-response
    (cl:cons ':comandC (comandC msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'collaborative_control_s)))
  'collaborative_control_s-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'collaborative_control_s)))
  'collaborative_control_s-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'collaborative_control_s)))
  "Returns string type for a service object of type '<collaborative_control_s>"
  "collaborative_control/collaborative_control_s")
