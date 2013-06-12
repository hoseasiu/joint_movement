
(cl:in-package :asdf)

(defsystem "joint_movement-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "joint_state" :depends-on ("_package_joint_state"))
    (:file "_package_joint_state" :depends-on ("_package"))
  ))