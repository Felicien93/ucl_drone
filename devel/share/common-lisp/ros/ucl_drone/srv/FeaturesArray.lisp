; Auto-generated. Do not edit!


(cl:in-package ucl_drone-srv)


;//! \htmlinclude FeaturesArray-request.msg.html

(cl:defclass <FeaturesArray-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass FeaturesArray-request (<FeaturesArray-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FeaturesArray-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FeaturesArray-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ucl_drone-srv:<FeaturesArray-request> is deprecated: use ucl_drone-srv:FeaturesArray-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FeaturesArray-request>) ostream)
  "Serializes a message object of type '<FeaturesArray-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FeaturesArray-request>) istream)
  "Deserializes a message object of type '<FeaturesArray-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FeaturesArray-request>)))
  "Returns string type for a service object of type '<FeaturesArray-request>"
  "ucl_drone/FeaturesArrayRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FeaturesArray-request)))
  "Returns string type for a service object of type 'FeaturesArray-request"
  "ucl_drone/FeaturesArrayRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FeaturesArray-request>)))
  "Returns md5sum for a message object of type '<FeaturesArray-request>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FeaturesArray-request)))
  "Returns md5sum for a message object of type 'FeaturesArray-request"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FeaturesArray-request>)))
  "Returns full string definition for message of type '<FeaturesArray-request>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FeaturesArray-request)))
  "Returns full string definition for message of type 'FeaturesArray-request"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FeaturesArray-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FeaturesArray-request>))
  "Converts a ROS message object to a list"
  (cl:list 'FeaturesArray-request
))
;//! \htmlinclude FeaturesArray-response.msg.html

(cl:defclass <FeaturesArray-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass FeaturesArray-response (<FeaturesArray-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FeaturesArray-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FeaturesArray-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ucl_drone-srv:<FeaturesArray-response> is deprecated: use ucl_drone-srv:FeaturesArray-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FeaturesArray-response>) ostream)
  "Serializes a message object of type '<FeaturesArray-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FeaturesArray-response>) istream)
  "Deserializes a message object of type '<FeaturesArray-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FeaturesArray-response>)))
  "Returns string type for a service object of type '<FeaturesArray-response>"
  "ucl_drone/FeaturesArrayResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FeaturesArray-response)))
  "Returns string type for a service object of type 'FeaturesArray-response"
  "ucl_drone/FeaturesArrayResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FeaturesArray-response>)))
  "Returns md5sum for a message object of type '<FeaturesArray-response>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FeaturesArray-response)))
  "Returns md5sum for a message object of type 'FeaturesArray-response"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FeaturesArray-response>)))
  "Returns full string definition for message of type '<FeaturesArray-response>"
  (cl:format cl:nil "~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FeaturesArray-response)))
  "Returns full string definition for message of type 'FeaturesArray-response"
  (cl:format cl:nil "~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FeaturesArray-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FeaturesArray-response>))
  "Converts a ROS message object to a list"
  (cl:list 'FeaturesArray-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'FeaturesArray)))
  'FeaturesArray-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'FeaturesArray)))
  'FeaturesArray-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FeaturesArray)))
  "Returns string type for a service object of type '<FeaturesArray>"
  "ucl_drone/FeaturesArray")