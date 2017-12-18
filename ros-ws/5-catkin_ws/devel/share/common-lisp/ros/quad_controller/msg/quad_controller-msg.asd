
(cl:in-package :asdf)

(defsystem "quad_controller-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "EulerAngles" :depends-on ("_package_EulerAngles"))
    (:file "_package_EulerAngles" :depends-on ("_package"))
  ))