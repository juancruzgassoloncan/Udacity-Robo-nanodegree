;; Auto-generated. Do not edit!


(when (boundp 'quad_controller::GetPath)
  (if (not (find-package "QUAD_CONTROLLER"))
    (make-package "QUAD_CONTROLLER"))
  (shadow 'GetPath (find-package "QUAD_CONTROLLER")))
(unless (find-package "QUAD_CONTROLLER::GETPATH")
  (make-package "QUAD_CONTROLLER::GETPATH"))
(unless (find-package "QUAD_CONTROLLER::GETPATHREQUEST")
  (make-package "QUAD_CONTROLLER::GETPATHREQUEST"))
(unless (find-package "QUAD_CONTROLLER::GETPATHRESPONSE")
  (make-package "QUAD_CONTROLLER::GETPATHRESPONSE"))

(in-package "ROS")



(if (not (find-package "NAV_MSGS"))
  (ros::roseus-add-msgs "nav_msgs"))


(defclass quad_controller::GetPathRequest
  :super ros::object
  :slots (_ignoreMe ))

(defmethod quad_controller::GetPathRequest
  (:init
   (&key
    ((:ignoreMe __ignoreMe) nil)
    )
   (send-super :init)
   (setq _ignoreMe __ignoreMe)
   self)
  (:ignoreMe
   (&optional __ignoreMe)
   (if __ignoreMe (setq _ignoreMe __ignoreMe)) _ignoreMe)
  (:serialization-length
   ()
   (+
    ;; bool _ignoreMe
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _ignoreMe
       (if _ignoreMe (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _ignoreMe
     (setq _ignoreMe (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass quad_controller::GetPathResponse
  :super ros::object
  :slots (_path ))

(defmethod quad_controller::GetPathResponse
  (:init
   (&key
    ((:path __path) (instance nav_msgs::Path :init))
    )
   (send-super :init)
   (setq _path __path)
   self)
  (:path
   (&rest __path)
   (if (keywordp (car __path))
       (send* _path __path)
     (progn
       (if __path (setq _path (car __path)))
       _path)))
  (:serialization-length
   ()
   (+
    ;; nav_msgs/Path _path
    (send _path :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; nav_msgs/Path _path
       (send _path :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; nav_msgs/Path _path
     (send _path :deserialize buf ptr-) (incf ptr- (send _path :serialization-length))
   ;;
   self)
  )

(defclass quad_controller::GetPath
  :super ros::object
  :slots ())

(setf (get quad_controller::GetPath :md5sum-) "f65fd00b2e3f675220a83a94307b7f12")
(setf (get quad_controller::GetPath :datatype-) "quad_controller/GetPath")
(setf (get quad_controller::GetPath :request) quad_controller::GetPathRequest)
(setf (get quad_controller::GetPath :response) quad_controller::GetPathResponse)

(defmethod quad_controller::GetPathRequest
  (:response () (instance quad_controller::GetPathResponse :init)))

(setf (get quad_controller::GetPathRequest :md5sum-) "f65fd00b2e3f675220a83a94307b7f12")
(setf (get quad_controller::GetPathRequest :datatype-) "quad_controller/GetPathRequest")
(setf (get quad_controller::GetPathRequest :definition-)
      "bool ignoreMe
---
nav_msgs/Path path

================================================================================
MSG: nav_msgs/Path
#An array of poses that represents a Path for a robot to follow
Header header
geometry_msgs/PoseStamped[] poses

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
")

(setf (get quad_controller::GetPathResponse :md5sum-) "f65fd00b2e3f675220a83a94307b7f12")
(setf (get quad_controller::GetPathResponse :datatype-) "quad_controller/GetPathResponse")
(setf (get quad_controller::GetPathResponse :definition-)
      "bool ignoreMe
---
nav_msgs/Path path

================================================================================
MSG: nav_msgs/Path
#An array of poses that represents a Path for a robot to follow
Header header
geometry_msgs/PoseStamped[] poses

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
")



(provide :quad_controller/GetPath "f65fd00b2e3f675220a83a94307b7f12")


