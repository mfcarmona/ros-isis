
(cl:in-package :asdf)

(defsystem "efficiency-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :efficiency-msg
               :geometry_msgs-msg
               :nav_msgs-msg
               :sensor_msgs-msg
)
  :components ((:file "_package")
    (:file "Efficiency_s" :depends-on ("_package_Efficiency_s"))
    (:file "_package_Efficiency_s" :depends-on ("_package"))
    (:file "Efficiency_s2" :depends-on ("_package_Efficiency_s2"))
    (:file "_package_Efficiency_s2" :depends-on ("_package"))
  ))