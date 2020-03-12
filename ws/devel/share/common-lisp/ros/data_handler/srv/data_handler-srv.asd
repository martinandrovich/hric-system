
(cl:in-package :asdf)

(defsystem "data_handler-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "RegisterHWNode" :depends-on ("_package_RegisterHWNode"))
    (:file "_package_RegisterHWNode" :depends-on ("_package"))
  ))