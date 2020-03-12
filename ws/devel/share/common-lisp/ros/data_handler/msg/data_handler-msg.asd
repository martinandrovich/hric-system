
(cl:in-package :asdf)

(defsystem "data_handler-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "GenericData" :depends-on ("_package_GenericData"))
    (:file "_package_GenericData" :depends-on ("_package"))
    (:file "TestMessage" :depends-on ("_package_TestMessage"))
    (:file "_package_TestMessage" :depends-on ("_package"))
  ))