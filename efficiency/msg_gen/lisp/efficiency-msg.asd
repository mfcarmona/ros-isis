
(cl:in-package :asdf)

(defsystem "efficiency-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Efficiency_m" :depends-on ("_package_Efficiency_m"))
    (:file "_package_Efficiency_m" :depends-on ("_package"))
  ))