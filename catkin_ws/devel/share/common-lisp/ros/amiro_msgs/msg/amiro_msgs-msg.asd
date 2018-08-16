
(cl:in-package :asdf)

(defsystem "amiro_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Int32MultiArrayStamped" :depends-on ("_package_Int32MultiArrayStamped"))
    (:file "_package_Int32MultiArrayStamped" :depends-on ("_package"))
    (:file "UInt16MultiArrayStamped" :depends-on ("_package_UInt16MultiArrayStamped"))
    (:file "_package_UInt16MultiArrayStamped" :depends-on ("_package"))
  ))