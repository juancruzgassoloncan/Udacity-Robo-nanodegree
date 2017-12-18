
(cl:in-package :asdf)

(defsystem "quad_controller-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :nav_msgs-msg
)
  :components ((:file "_package")
    (:file "GetPath" :depends-on ("_package_GetPath"))
    (:file "_package_GetPath" :depends-on ("_package"))
    (:file "SetFloat" :depends-on ("_package_SetFloat"))
    (:file "_package_SetFloat" :depends-on ("_package"))
    (:file "SetInt" :depends-on ("_package_SetInt"))
    (:file "_package_SetInt" :depends-on ("_package"))
    (:file "SetPath" :depends-on ("_package_SetPath"))
    (:file "_package_SetPath" :depends-on ("_package"))
    (:file "SetPose" :depends-on ("_package_SetPose"))
    (:file "_package_SetPose" :depends-on ("_package"))
  ))