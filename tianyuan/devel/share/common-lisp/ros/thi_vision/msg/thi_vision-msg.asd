
(cl:in-package :asdf)

(defsystem "thi_vision-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "orientation" :depends-on ("_package_orientation"))
    (:file "_package_orientation" :depends-on ("_package"))
    (:file "pcArea" :depends-on ("_package_pcArea"))
    (:file "_package_pcArea" :depends-on ("_package"))
    (:file "pose" :depends-on ("_package_pose"))
    (:file "_package_pose" :depends-on ("_package"))
    (:file "position" :depends-on ("_package_position"))
    (:file "_package_position" :depends-on ("_package"))
    (:file "track" :depends-on ("_package_track"))
    (:file "_package_track" :depends-on ("_package"))
  ))