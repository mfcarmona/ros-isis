
(cl:in-package :asdf)

(defsystem "collaborative_control-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :efficiency-msg
               :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "collaborative_control_s" :depends-on ("_package_collaborative_control_s"))
    (:file "_package_collaborative_control_s" :depends-on ("_package"))
  ))