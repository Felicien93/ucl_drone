; Auto-generated. Do not edit!


(cl:in-package ucl_drone-srv)


;//! \htmlinclude SetPoseRef-request.msg.html

(cl:defclass <SetPoseRef-request> (roslisp-msg-protocol:ros-message)
  ((x
    :reader x
    :initarg :x
    :type cl:float
    :initform 0.0)
   (y
    :reader y
    :initarg :y
    :type cl:float
    :initform 0.0)
   (z
    :reader z
    :initarg :z
    :type cl:float
    :initform 0.0)
   (yaw
    :reader yaw
    :initarg :yaw
    :type cl:float
    :initform 0.0))
)

(cl:defclass SetPoseRef-request (<SetPoseRef-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetPoseRef-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetPoseRef-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ucl_drone-srv:<SetPoseRef-request> is deprecated: use ucl_drone-srv:SetPoseRef-request instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <SetPoseRef-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ucl_drone-srv:x-val is deprecated.  Use ucl_drone-srv:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <SetPoseRef-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ucl_drone-srv:y-val is deprecated.  Use ucl_drone-srv:y instead.")
  (y m))

(cl:ensure-generic-function 'z-val :lambda-list '(m))
(cl:defmethod z-val ((m <SetPoseRef-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ucl_drone-srv:z-val is deprecated.  Use ucl_drone-srv:z instead.")
  (z m))

(cl:ensure-generic-function 'yaw-val :lambda-list '(m))
(cl:defmethod yaw-val ((m <SetPoseRef-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ucl_drone-srv:yaw-val is deprecated.  Use ucl_drone-srv:yaw instead.")
  (yaw m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetPoseRef-request>) ostream)
  "Serializes a message object of type '<SetPoseRef-request>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'yaw))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetPoseRef-request>) istream)
  "Deserializes a message object of type '<SetPoseRef-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'z) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'yaw) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetPoseRef-request>)))
  "Returns string type for a service object of type '<SetPoseRef-request>"
  "ucl_drone/SetPoseRefRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetPoseRef-request)))
  "Returns string type for a service object of type 'SetPoseRef-request"
  "ucl_drone/SetPoseRefRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetPoseRef-request>)))
  "Returns md5sum for a message object of type '<SetPoseRef-request>"
  "259d6d91085aaefc76451f0ab02bff43")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetPoseRef-request)))
  "Returns md5sum for a message object of type 'SetPoseRef-request"
  "259d6d91085aaefc76451f0ab02bff43")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetPoseRef-request>)))
  "Returns full string definition for message of type '<SetPoseRef-request>"
  (cl:format cl:nil "~%float32 x~%float32 y~%float32 z~%float32 yaw~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetPoseRef-request)))
  "Returns full string definition for message of type 'SetPoseRef-request"
  (cl:format cl:nil "~%float32 x~%float32 y~%float32 z~%float32 yaw~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetPoseRef-request>))
  (cl:+ 0
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetPoseRef-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetPoseRef-request
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':z (z msg))
    (cl:cons ':yaw (yaw msg))
))
;//! \htmlinclude SetPoseRef-response.msg.html

(cl:defclass <SetPoseRef-response> (roslisp-msg-protocol:ros-message)
  ((status
    :reader status
    :initarg :status
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetPoseRef-response (<SetPoseRef-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetPoseRef-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetPoseRef-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ucl_drone-srv:<SetPoseRef-response> is deprecated: use ucl_drone-srv:SetPoseRef-response instead.")))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <SetPoseRef-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ucl_drone-srv:status-val is deprecated.  Use ucl_drone-srv:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetPoseRef-response>) ostream)
  "Serializes a message object of type '<SetPoseRef-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'status) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetPoseRef-response>) istream)
  "Deserializes a message object of type '<SetPoseRef-response>"
    (cl:setf (cl:slot-value msg 'status) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetPoseRef-response>)))
  "Returns string type for a service object of type '<SetPoseRef-response>"
  "ucl_drone/SetPoseRefResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetPoseRef-response)))
  "Returns string type for a service object of type 'SetPoseRef-response"
  "ucl_drone/SetPoseRefResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetPoseRef-response>)))
  "Returns md5sum for a message object of type '<SetPoseRef-response>"
  "259d6d91085aaefc76451f0ab02bff43")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetPoseRef-response)))
  "Returns md5sum for a message object of type 'SetPoseRef-response"
  "259d6d91085aaefc76451f0ab02bff43")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetPoseRef-response>)))
  "Returns full string definition for message of type '<SetPoseRef-response>"
  (cl:format cl:nil "~%bool status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetPoseRef-response)))
  "Returns full string definition for message of type 'SetPoseRef-response"
  (cl:format cl:nil "~%bool status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetPoseRef-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetPoseRef-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetPoseRef-response
    (cl:cons ':status (status msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetPoseRef)))
  'SetPoseRef-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetPoseRef)))
  'SetPoseRef-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetPoseRef)))
  "Returns string type for a service object of type '<SetPoseRef>"
  "ucl_drone/SetPoseRef")