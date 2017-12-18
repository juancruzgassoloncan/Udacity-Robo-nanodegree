;; Auto-generated. Do not edit!


(when (boundp 'quad_controller::SetFloat)
  (if (not (find-package "QUAD_CONTROLLER"))
    (make-package "QUAD_CONTROLLER"))
  (shadow 'SetFloat (find-package "QUAD_CONTROLLER")))
(unless (find-package "QUAD_CONTROLLER::SETFLOAT")
  (make-package "QUAD_CONTROLLER::SETFLOAT"))
(unless (find-package "QUAD_CONTROLLER::SETFLOATREQUEST")
  (make-package "QUAD_CONTROLLER::SETFLOATREQUEST"))
(unless (find-package "QUAD_CONTROLLER::SETFLOATRESPONSE")
  (make-package "QUAD_CONTROLLER::SETFLOATRESPONSE"))

(in-package "ROS")





(defclass quad_controller::SetFloatRequest
  :super ros::object
  :slots (_data ))

(defmethod quad_controller::SetFloatRequest
  (:init
   (&key
    ((:data __data) 0.0)
    )
   (send-super :init)
   (setq _data (float __data))
   self)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:serialization-length
   ()
   (+
    ;; float32 _data
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _data
       (sys::poke _data (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _data
     (setq _data (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass quad_controller::SetFloatResponse
  :super ros::object
  :slots (_success _newData ))

(defmethod quad_controller::SetFloatResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:newData __newData) 0.0)
    )
   (send-super :init)
   (setq _success __success)
   (setq _newData (float __newData))
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
    ;; float32 _newData
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; float32 _newData
       (sys::poke _newData (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float32 _newData
     (setq _newData (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass quad_controller::SetFloat
  :super ros::object
  :slots ())

(setf (get quad_controller::SetFloat :md5sum-) "33440febf8aeff64c8c846264db06c0f")
(setf (get quad_controller::SetFloat :datatype-) "quad_controller/SetFloat")
(setf (get quad_controller::SetFloat :request) quad_controller::SetFloatRequest)
(setf (get quad_controller::SetFloat :response) quad_controller::SetFloatResponse)

(defmethod quad_controller::SetFloatRequest
  (:response () (instance quad_controller::SetFloatResponse :init)))

(setf (get quad_controller::SetFloatRequest :md5sum-) "33440febf8aeff64c8c846264db06c0f")
(setf (get quad_controller::SetFloatRequest :datatype-) "quad_controller/SetFloatRequest")
(setf (get quad_controller::SetFloatRequest :definition-)
      "float32 data
---
bool success
float32 newData
")

(setf (get quad_controller::SetFloatResponse :md5sum-) "33440febf8aeff64c8c846264db06c0f")
(setf (get quad_controller::SetFloatResponse :datatype-) "quad_controller/SetFloatResponse")
(setf (get quad_controller::SetFloatResponse :definition-)
      "float32 data
---
bool success
float32 newData
")



(provide :quad_controller/SetFloat "33440febf8aeff64c8c846264db06c0f")


