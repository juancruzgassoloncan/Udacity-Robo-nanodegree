; Auto-generated. Do not edit!


(cl:in-package quad_controller-srv)


;//! \htmlinclude SetPath-request.msg.html

(cl:defclass <SetPath-request> (roslisp-msg-protocol:ros-message)
  ((path
    :reader path
    :initarg :path
    :type nav_msgs-msg:Path
    :initform (cl:make-instance 'nav_msgs-msg:Path)))
)

(cl:defclass SetPath-request (<SetPath-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetPath-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetPath-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name quad_controller-srv:<SetPath-request> is deprecated: use quad_controller-srv:SetPath-request instead.")))

(cl:ensure-generic-function 'path-val :lambda-list '(m))
(cl:defmethod path-val ((m <SetPath-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quad_controller-srv:path-val is deprecated.  Use quad_controller-srv:path instead.")
  (path m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetPath-request>) ostream)
  "Serializes a message object of type '<SetPath-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'path) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetPath-request>) istream)
  "Deserializes a message object of type '<SetPath-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'path) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetPath-request>)))
  "Returns string type for a service object of type '<SetPath-request>"
  "quad_controller/SetPathRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetPath-request)))
  "Returns string type for a service object of type 'SetPath-request"
  "quad_controller/SetPathRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetPath-request>)))
  "Returns md5sum for a message object of type '<SetPath-request>"
  "1b4da6832b76c5bb61a99129a3adb7b2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetPath-request)))
  "Returns md5sum for a message object of type 'SetPath-request"
  "1b4da6832b76c5bb61a99129a3adb7b2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetPath-request>)))
  "Returns full string definition for message of type '<SetPath-request>"
  (cl:format cl:nil "nav_msgs/Path path~%~%================================================================================~%MSG: nav_msgs/Path~%#An array of poses that represents a Path for a robot to follow~%Header header~%geometry_msgs/PoseStamped[] poses~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetPath-request)))
  "Returns full string definition for message of type 'SetPath-request"
  (cl:format cl:nil "nav_msgs/Path path~%~%================================================================================~%MSG: nav_msgs/Path~%#An array of poses that represents a Path for a robot to follow~%Header header~%geometry_msgs/PoseStamped[] poses~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetPath-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'path))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetPath-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetPath-request
    (cl:cons ':path (path msg))
))
;//! \htmlinclude SetPath-response.msg.html

(cl:defclass <SetPath-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetPath-response (<SetPath-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetPath-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetPath-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name quad_controller-srv:<SetPath-response> is deprecated: use quad_controller-srv:SetPath-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetPath-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quad_controller-srv:success-val is deprecated.  Use quad_controller-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetPath-response>) ostream)
  "Serializes a message object of type '<SetPath-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetPath-response>) istream)
  "Deserializes a message object of type '<SetPath-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetPath-response>)))
  "Returns string type for a service object of type '<SetPath-response>"
  "quad_controller/SetPathResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetPath-response)))
  "Returns string type for a service object of type 'SetPath-response"
  "quad_controller/SetPathResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetPath-response>)))
  "Returns md5sum for a message object of type '<SetPath-response>"
  "1b4da6832b76c5bb61a99129a3adb7b2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetPath-response)))
  "Returns md5sum for a message object of type 'SetPath-response"
  "1b4da6832b76c5bb61a99129a3adb7b2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetPath-response>)))
  "Returns full string definition for message of type '<SetPath-response>"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetPath-response)))
  "Returns full string definition for message of type 'SetPath-response"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetPath-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetPath-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetPath-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetPath)))
  'SetPath-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetPath)))
  'SetPath-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetPath)))
  "Returns string type for a service object of type '<SetPath>"
  "quad_controller/SetPath")