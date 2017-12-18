;; Auto-generated. Do not edit!


(when (boundp 'quad_controller::SetPath)
  (if (not (find-package "QUAD_CONTROLLER"))
    (make-package "QUAD_CONTROLLER"))
  (shadow 'SetPath (find-package "QUAD_CONTROLLER")))
(unless (find-package "QUAD_CONTROLLER::SETPATH")
  (make-package "QUAD_CONTROLLER::SETPATH"))
(unless (find-package "QUAD_CONTROLLER::SETPATHREQUEST")
  (make-package "QUAD_CONTROLLER::SETPATHREQUEST"))
(unless (find-package "QUAD_CONTROLLER::SETPATHRESPONSE")
  (make-package "QUAD_CONTROLLER::SETPATHRESPONSE"))

(in-package "ROS")

(if (not (find-package "NAV_MSGS"))
  (ros::roseus-add-msgs "nav_msgs"))




(defclass quad_controller::SetPathRequest
  :super ros::object
  :slots (_path ))

(defmethod quad_controller::SetPathRequest
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

(defclass quad_controller::SetPathResponse
  :super ros::object
  :slots (_success ))

(defmethod quad_controller::SetPathResponse
  (:init
   (&key
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _success __success)
   self)
  (:success
   (&optional __success)
   (if __success (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass quad_controller::SetPath
  :super ros::object
  :slots ())

(setf (get quad_controller::SetPath :md5sum-) "1b4da6832b76c5bb61a99129a3adb7b2")
(setf (get quad_controller::SetPath :datatype-) "quad_controller/SetPath")
(setf (get quad_controller::SetPath :request) quad_controller::SetPathRequest)
(setf (get quad_controller::SetPath :response) quad_controller::SetPathResponse)

(defmethod quad_controller::SetPathRequest
  (:response () (instance quad_controller::SetPathResponse :init)))

(setf (get quad_controller::SetPathRequest :md5sum-) "1b4da6832b76c5bb61a99129a3adb7b2")
(setf (get quad_controller::SetPathRequest :datatype-) "quad_controller/SetPathRequest")
(setf (get quad_controller::SetPathRequest :definition-)
      "nav_msgs/Path path

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
---
bool success
")

(setf (get quad_controller::SetPathResponse :md5sum-) "1b4da6832b76c5bb61a99129a3adb7b2")
(setf (get quad_controller::SetPathResponse :datatype-) "quad_controller/SetPathResponse")
(setf (get quad_controller::SetPathResponse :definition-)
      "nav_msgs/Path path

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
---
bool success
")



(provide :quad_controller/SetPath "1b4da6832b76c5bb61a99129a3adb7b2")


