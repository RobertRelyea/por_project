
(cl:in-package :asdf)

(defsystem "dart_delivery-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "RosieOdom" :depends-on ("_package_RosieOdom"))
    (:file "_package_RosieOdom" :depends-on ("_package"))
  ))