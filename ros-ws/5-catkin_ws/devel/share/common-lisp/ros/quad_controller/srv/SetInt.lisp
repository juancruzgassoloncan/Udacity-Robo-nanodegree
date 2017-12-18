; Auto-generated. Do not edit!


(cl:in-package quad_controller-srv)


;//! \htmlinclude SetInt-request.msg.html

(cl:defclass <SetInt-request> (roslisp-msg-protocol:ros-message)
  ((data
    :reader data
    :initarg :data
    :type cl:integer
    :initform 0))
)

(cl:defclass SetInt-request (<SetInt-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetInt-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetInt-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name quad_controller-srv:<SetInt-request> is deprecated: use quad_controller-srv:SetInt-request instead.")))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <SetInt-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quad_controller-srv:data-val is deprecated.  Use quad_controller-srv:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetInt-request>) ostream)
  "Serializes a message object of type '<SetInt-request>"
  (cl:let* ((signed (cl:slot-value msg 'data)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetInt-request>) istream)
  "Deserializes a message object of type '<SetInt-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'data) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetInt-request>)))
  "Returns string type for a service object of type '<SetInt-request>"
  "quad_controller/SetIntRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetInt-request)))
  "Returns string type for a service object of type 'SetInt-request"
  "quad_controller/SetIntRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetInt-request>)))
  "Returns md5sum for a message object of type '<SetInt-request>"
  "c6cf86a08ff7309c5d9f200d8652f57f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetInt-request)))
  "Returns md5sum for a message object of type 'SetInt-request"
  "c6cf86a08ff7309c5d9f200d8652f57f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetInt-request>)))
  "Returns full string definition for message of type '<SetInt-request>"
  (cl:format cl:nil "int32 data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetInt-request)))
  "Returns full string definition for message of type 'SetInt-request"
  (cl:format cl:nil "int32 data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetInt-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetInt-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetInt-request
    (cl:cons ':data (data msg))
))
;//! \htmlinclude SetInt-response.msg.html

(cl:defclass <SetInt-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (newData
    :reader newData
    :initarg :newData
    :type cl:integer
    :initform 0))
)

(cl:defclass SetInt-response (<SetInt-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetInt-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetInt-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name quad_controller-srv:<SetInt-response> is deprecated: use quad_controller-srv:SetInt-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetInt-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quad_controller-srv:success-val is deprecated.  Use quad_controller-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'newData-val :lambda-list '(m))
(cl:defmethod newData-val ((m <SetInt-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quad_controller-srv:newData-val is deprecated.  Use quad_controller-srv:newData instead.")
  (newData m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetInt-response>) ostream)
  "Serializes a message object of type '<SetInt-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'newData)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetInt-response>) istream)
  "Deserializes a message object of type '<SetInt-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'newData) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetInt-response>)))
  "Returns string type for a service object of type '<SetInt-response>"
  "quad_controller/SetIntResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetInt-response)))
  "Returns string type for a service object of type 'SetInt-response"
  "quad_controller/SetIntResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetInt-response>)))
  "Returns md5sum for a message object of type '<SetInt-response>"
  "c6cf86a08ff7309c5d9f200d8652f57f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetInt-response)))
  "Returns md5sum for a message object of type 'SetInt-response"
  "c6cf86a08ff7309c5d9f200d8652f57f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetInt-response>)))
  "Returns full string definition for message of type '<SetInt-response>"
  (cl:format cl:nil "bool success~%int32 newData~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetInt-response)))
  "Returns full string definition for message of type 'SetInt-response"
  (cl:format cl:nil "bool success~%int32 newData~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetInt-response>))
  (cl:+ 0
     1
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetInt-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetInt-response
    (cl:cons ':success (success msg))
    (cl:cons ':newData (newData msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetInt)))
  'SetInt-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetInt)))
  'SetInt-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetInt)))
  "Returns string type for a service object of type '<SetInt>"
  "quad_controller/SetInt")