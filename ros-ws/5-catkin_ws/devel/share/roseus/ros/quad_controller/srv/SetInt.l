;; Auto-generated. Do not edit!


(when (boundp 'quad_controller::SetInt)
  (if (not (find-package "QUAD_CONTROLLER"))
    (make-package "QUAD_CONTROLLER"))
  (shadow 'SetInt (find-package "QUAD_CONTROLLER")))
(unless (find-package "QUAD_CONTROLLER::SETINT")
  (make-package "QUAD_CONTROLLER::SETINT"))
(unless (find-package "QUAD_CONTROLLER::SETINTREQUEST")
  (make-package "QUAD_CONTROLLER::SETINTREQUEST"))
(unless (find-package "QUAD_CONTROLLER::SETINTRESPONSE")
  (make-package "QUAD_CONTROLLER::SETINTRESPONSE"))

(in-package "ROS")





(defclass quad_controller::SetIntRequest
  :super ros::object
  :slots (_data ))

(defmethod quad_controller::SetIntRequest
  (:init
   (&key
    ((:data __data) 0)
    )
   (send-super :init)
   (setq _data (round __data))
   self)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:serialization-length
   ()
   (+
    ;; int32 _data
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _data
       (write-long _data s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _data
     (setq _data (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass quad_controller::SetIntResponse
  :super ros::object
  :slots (_success _newData ))

(defmethod quad_controller::SetIntResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:newData __newData) 0)
    )
   (send-super :init)
   (setq _success __success)
   (setq _newData (round __newData))
   self)
  (:success
   (&optional __success)
   (if __success (setq _success __success)) _success)
  (:newData
   (&optional __newData)
   (if __newData (setq _newData __newData)) _newData)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; int32 _newData
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; int32 _newData
       (write-long _newData s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; int32 _newData
     (setq _newData (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass quad_controller::SetInt
  :super ros::object
  :slots ())

(setf (get quad_controller::SetInt :md5sum-) "c6cf86a08ff7309c5d9f200d8652f57f")
(setf (get quad_controller::SetInt :datatype-) "quad_controller/SetInt")
(setf (get quad_controller::SetInt :request) quad_controller::SetIntRequest)
(setf (get quad_controller::SetInt :response) quad_controller::SetIntResponse)

(defmethod quad_controller::SetIntRequest
  (:response () (instance quad_controller::SetIntResponse :init)))

(setf (get quad_controller::SetIntRequest :md5sum-) "c6cf86a08ff7309c5d9f200d8652f57f")
(setf (get quad_controller::SetIntRequest :datatype-) "quad_controller/SetIntRequest")
(setf (get quad_controller::SetIntRequest :definition-)
      "int32 data
---
bool success
int32 newData
")

(setf (get quad_controller::SetIntResponse :md5sum-) "c6cf86a08ff7309c5d9f200d8652f57f")
(setf (get quad_controller::SetIntResponse :datatype-) "quad_controller/SetIntResponse")
(setf (get quad_controller::SetIntResponse :definition-)
      "int32 data
---
bool success
int32 newData
")



(provide :quad_controller/SetInt "c6cf86a08ff7309c5d9f200d8652f57f")


