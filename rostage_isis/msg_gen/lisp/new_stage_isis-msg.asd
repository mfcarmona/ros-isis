
(cl:in-package :asdf)

(defsystem "new_stage_isis-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Range" :depends-on ("_package_Range"))
    (:file "_package_Range" :depends-on ("_package"))
  ))