; Auto-generated. Do not edit!


(cl:in-package ucl_drone-srv)


;//! \htmlinclude Projection-request.msg.html

(cl:defclass <Projection-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass Projection-request (<Projection-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Projection-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Projection-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ucl_drone-srv:<Projection-request> is deprecated: use ucl_drone-srv:Projection-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Projection-request>) ostream)
  "Serializes a message object of type '<Projection-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Projection-request>) istream)
  "Deserializes a message object of type '<Projection-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Projection-request>)))
  "Returns string type for a service object of type '<Projection-request>"
  "ucl_drone/ProjectionRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Projection-request)))
  "Returns string type for a service object of type 'Projection-request"
  "ucl_drone/ProjectionRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Projection-request>)))
  "Returns md5sum for a message object of type '<Projection-request>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Projection-request)))
  "Returns md5sum for a message object of type 'Projection-request"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Projection-request>)))
  "Returns full string definition for message of type '<Projection-request>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Projection-request)))
  "Returns full string definition for message of type 'Projection-request"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Projection-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Projection-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Projection-request
))
;//! \htmlinclude Projection-response.msg.html

(cl:defclass <Projection-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass Projection-response (<Projection-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Projection-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Projection-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ucl_drone-srv:<Projection-response> is deprecated: use ucl_drone-srv:Projection-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Projection-response>) ostream)
  "Serializes a message object of type '<Projection-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Projection-response>) istream)
  "Deserializes a message object of type '<Projection-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Projection-response>)))
  "Returns string type for a service object of type '<Projection-response>"
  "ucl_drone/ProjectionResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Projection-response)))
  "Returns string type for a service object of type 'Projection-response"
  "ucl_drone/ProjectionResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Projection-response>)))
  "Returns md5sum for a message object of type '<Projection-response>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Projection-response)))
  "Returns md5sum for a message object of type 'Projection-response"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Projection-response>)))
  "Returns full string definition for message of type '<Projection-response>"
  (cl:format cl:nil "~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Projection-response)))
  "Returns full string definition for message of type 'Projection-response"
  (cl:format cl:nil "~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Projection-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Projection-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Projection-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Projection)))
  'Projection-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Projection)))
  'Projection-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Projection)))
  "Returns string type for a service object of type '<Projection>"
  "ucl_drone/Projection")