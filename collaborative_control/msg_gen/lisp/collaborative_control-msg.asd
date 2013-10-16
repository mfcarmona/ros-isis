
(cl:in-package :asdf)

(defsystem "collaborative_control-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :efficiency-msg
               :geometry_msgs-msg
               :nav_msgs-msg
)
  :components ((:file "_package")
    (:file "cocon_m" :depends-on ("_package_cocon_m"))
    (:file "_package_cocon_m" :depends-on ("_package"))
  ))