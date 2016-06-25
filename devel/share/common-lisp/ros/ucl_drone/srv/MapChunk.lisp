; Auto-generated. Do not edit!


(cl:in-package ucl_drone-srv)


;//! \htmlinclude MapChunk-request.msg.html

(cl:defclass <MapChunk-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass MapChunk-request (<MapChunk-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MapChunk-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MapChunk-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ucl_drone-srv:<MapChunk-request> is deprecated: use ucl_drone-srv:MapChunk-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MapChunk-request>) ostream)
  "Serializes a message object of type '<MapChunk-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MapChunk-request>) istream)
  "Deserializes a message object of type '<MapChunk-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MapChunk-request>)))
  "Returns string type for a service object of type '<MapChunk-request>"
  "ucl_drone/MapChunkRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MapChunk-request)))
  "Returns string type for a service object of type 'MapChunk-request"
  "ucl_drone/MapChunkRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MapChunk-request>)))
  "Returns md5sum for a message object of type '<MapChunk-request>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MapChunk-request)))
  "Returns md5sum for a message object of type 'MapChunk-request"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MapChunk-request>)))
  "Returns full string definition for message of type '<MapChunk-request>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MapChunk-request)))
  "Returns full string definition for message of type 'MapChunk-request"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MapChunk-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MapChunk-request>))
  "Converts a ROS message object to a list"
  (cl:list 'MapChunk-request
))
;//! \htmlinclude MapChunk-response.msg.html

(cl:defclass <MapChunk-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass MapChunk-response (<MapChunk-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MapChunk-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MapChunk-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ucl_drone-srv:<MapChunk-response> is deprecated: use ucl_drone-srv:MapChunk-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MapChunk-response>) ostream)
  "Serializes a message object of type '<MapChunk-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MapChunk-response>) istream)
  "Deserializes a message object of type '<MapChunk-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MapChunk-response>)))
  "Returns string type for a service object of type '<MapChunk-response>"
  "ucl_drone/MapChunkResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MapChunk-response)))
  "Returns string type for a service object of type 'MapChunk-response"
  "ucl_drone/MapChunkResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MapChunk-response>)))
  "Returns md5sum for a message object of type '<MapChunk-response>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MapChunk-response)))
  "Returns md5sum for a message object of type 'MapChunk-response"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MapChunk-response>)))
  "Returns full string definition for message of type '<MapChunk-response>"
  (cl:format cl:nil "~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MapChunk-response)))
  "Returns full string definition for message of type 'MapChunk-response"
  (cl:format cl:nil "~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MapChunk-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MapChunk-response>))
  "Converts a ROS message object to a list"
  (cl:list 'MapChunk-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'MapChunk)))
  'MapChunk-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'MapChunk)))
  'MapChunk-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MapChunk)))
  "Returns string type for a service object of type '<MapChunk>"
  "ucl_drone/MapChunk")