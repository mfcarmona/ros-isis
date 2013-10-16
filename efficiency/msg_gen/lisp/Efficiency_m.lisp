; Auto-generated. Do not edit!


(cl:in-package efficiency-msg)


;//! \htmlinclude Efficiency_m.msg.html

(cl:defclass <Efficiency_m> (roslisp-msg-protocol:ros-message)
  ((Global
    :reader Global
    :initarg :Global
    :type cl:float
    :initform 0.0)
   (Safety
    :reader Safety
    :initarg :Safety
    :type cl:float
    :initform 0.0)
   (Directness
    :reader Directness
    :initarg :Directness
    :type cl:float
    :initform 0.0)
   (Smoothness
    :reader Smoothness
    :initarg :Smoothness
    :type cl:float
    :initform 0.0))
)

(cl:defclass Efficiency_m (<Efficiency_m>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Efficiency_m>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Efficiency_m)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name efficiency-msg:<Efficiency_m> is deprecated: use efficiency-msg:Efficiency_m instead.")))

(cl:ensure-generic-function 'Global-val :lambda-list '(m))
(cl:defmethod Global-val ((m <Efficiency_m>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader efficiency-msg:Global-val is deprecated.  Use efficiency-msg:Global instead.")
  (Global m))

(cl:ensure-generic-function 'Safety-val :lambda-list '(m))
(cl:defmethod Safety-val ((m <Efficiency_m>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader efficiency-msg:Safety-val is deprecated.  Use efficiency-msg:Safety instead.")
  (Safety m))

(cl:ensure-generic-function 'Directness-val :lambda-list '(m))
(cl:defmethod Directness-val ((m <Efficiency_m>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader efficiency-msg:Directness-val is deprecated.  Use efficiency-msg:Directness instead.")
  (Directness m))

(cl:ensure-generic-function 'Smoothness-val :lambda-list '(m))
(cl:defmethod Smoothness-val ((m <Efficiency_m>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader efficiency-msg:Smoothness-val is deprecated.  Use efficiency-msg:Smoothness instead.")
  (Smoothness m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Efficiency_m>) ostream)
  "Serializes a message object of type '<Efficiency_m>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'Global))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'Safety))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'Directness))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'Smoothness))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Efficiency_m>) istream)
  "Deserializes a message object of type '<Efficiency_m>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'Global) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'Safety) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'Directness) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'Smoothness) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Efficiency_m>)))
  "Returns string type for a message object of type '<Efficiency_m>"
  "efficiency/Efficiency_m")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Efficiency_m)))
  "Returns string type for a message object of type 'Efficiency_m"
  "efficiency/Efficiency_m")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Efficiency_m>)))
  "Returns md5sum for a message object of type '<Efficiency_m>"
  "0c310688c11a3495d825779498ee3fd4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Efficiency_m)))
  "Returns md5sum for a message object of type 'Efficiency_m"
  "0c310688c11a3495d825779498ee3fd4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Efficiency_m>)))
  "Returns full string definition for message of type '<Efficiency_m>"
  (cl:format cl:nil "float32 Global~%float32 Safety~%float32 Directness~%float32 Smoothness~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Efficiency_m)))
  "Returns full string definition for message of type 'Efficiency_m"
  (cl:format cl:nil "float32 Global~%float32 Safety~%float32 Directness~%float32 Smoothness~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Efficiency_m>))
  (cl:+ 0
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Efficiency_m>))
  "Converts a ROS message object to a list"
  (cl:list 'Efficiency_m
    (cl:cons ':Global (Global msg))
    (cl:cons ':Safety (Safety msg))
    (cl:cons ':Directness (Directness msg))
    (cl:cons ':Smoothness (Smoothness msg))
))
