
(cl:in-package :asdf)

(defsystem "kuka_tcp-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :kuka_tcp-msg
)
  :components ((:file "_package")
    (:file "kukaTrack" :depends-on ("_package_kukaTrack"))
    (:file "_package_kukaTrack" :depends-on ("_package"))
  ))