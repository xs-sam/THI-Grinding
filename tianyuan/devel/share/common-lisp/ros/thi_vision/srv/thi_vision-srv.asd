
(cl:in-package :asdf)

(defsystem "thi_vision-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :thi_vision-msg
)
  :components ((:file "_package")
    (:file "visionTracks" :depends-on ("_package_visionTracks"))
    (:file "_package_visionTracks" :depends-on ("_package"))
  ))