; Auto-generated. Do not edit!


(cl:in-package quad_controller-srv)


;//! \htmlinclude GetPath-request.msg.html

(cl:defclass <GetPath-request> (roslisp-msg-protocol:ros-message)
  ((ignoreMe
    :reader ignoreMe
    :initarg :ignoreMe
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass GetPath-request (<GetPath-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetPath-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetPath-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name quad_controller-srv:<GetPath-request> is deprecated: use quad_controller-srv:GetPath-request instead.")))

(cl:ensure-generic-function 'ignoreMe-val :lambda-list '(m))
(cl:defmethod ignoreMe-val ((m <GetPath-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quad_controller-srv:ignoreMe-val is deprecated.  Use quad_controller-srv:ignoreMe instead.")
  (ignoreMe m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetPath-request>) ostream)
  "Serializes a message object of type '<GetPath-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ignoreMe) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetPath-request>) istream)
  "Deserializes a message object of type '<GetPath-request>"
    (cl:setf (cl:slot-value msg 'ignoreMe) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetPath-request>)))
  "Returns string type for a service object of type '<GetPath-request>"
  "quad_controller/GetPathRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetPath-request)))
  "Returns string type for a service object of type 'GetPath-request"
  "quad_controller/GetPathRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetPath-request>)))
  "Returns md5sum for a message object of type '<GetPath-request>"
  "f65fd00b2e3f675220a83a94307b7f12")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetPath-request)))
  "Returns md5sum for a message object of type 'GetPath-request"
  "f65fd00b2e3f675220a83a94307b7f12")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetPath-request>)))
  "Returns full string definition for message of type '<GetPath-request>"
  (cl:format cl:nil "bool ignoreMe~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetPath-request)))
  "Returns full string definition for message of type 'GetPath-request"
  (cl:format cl:nil "bool ignoreMe~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetPath-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetPath-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetPath-request
    (cl:cons ':ignoreMe (ignoreMe msg))
))
;//! \htmlinclude GetPath-response.msg.html

(cl:defclass <GetPath-response> (roslisp-msg-protocol:ros-message)
  ((path
    :reader path
    :initarg :path
    :type nav_msgs-msg:Path
    :initform (cl:make-instance 'nav_msgs-msg:Path)))
)

(cl:defclass GetPath-response (<GetPath-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetPath-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetPath-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name quad_controller-srv:<GetPath-response> is deprecated: use quad_controller-srv:GetPath-response instead.")))

(cl:ensure-generic-function 'path-val :lambda-list '(m))
(cl:defmethod path-val ((m <GetPath-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quad_controller-srv:path-val is deprecated.  Use quad_controller-srv:path instead.")
  (path m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetPath-response>) ostream)
  "Serializes a message object of type '<GetPath-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'path) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetPath-response>) istream)
  "Deserializes a message object of type '<GetPath-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'path) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetPath-response>)))
  "Returns string type for a service object of type '<GetPath-response>"
  "quad_controller/GetPathResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetPath-response)))
  "Returns string type for a service object of type 'GetPath-response"
  "quad_controller/GetPathResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetPath-response>)))
  "Returns md5sum for a message object of type '<GetPath-response>"
  "f65fd00b2e3f675220a83a94307b7f12")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetPath-response)))
  "Returns md5sum for a message object of type 'GetPath-response"
  "f65fd00b2e3f675220a83a94307b7f12")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetPath-response>)))
  "Returns full string definition for message of type '<GetPath-response>"
  (cl:format cl:nil "nav_msgs/Path path~%~%================================================================================~%MSG: nav_msgs/Path~%#An array of poses that represents a Path for a robot to follow~%Header header~%geometry_msgs/PoseStamped[] poses~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetPath-response)))
  "Returns full string definition for message of type 'GetPath-response"
  (cl:format cl:nil "nav_msgs/Path path~%~%================================================================================~%MSG: nav_msgs/Path~%#An array of poses that represents a Path for a robot to follow~%Header header~%geometry_msgs/PoseStamped[] poses~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetPath-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'path))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetPath-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetPath-response
    (cl:cons ':path (path msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetPath)))
  'GetPath-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetPath)))
  'GetPath-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetPath)))
  "Returns string type for a service object of type '<GetPath>"
  "quad_controller/GetPath")