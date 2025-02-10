
(cl:in-package :asdf)

(defsystem "kuka_tcp-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "kukaPoint" :depends-on ("_package_kukaPoint"))
    (:file "_package_kukaPoint" :depends-on ("_package"))
  ))