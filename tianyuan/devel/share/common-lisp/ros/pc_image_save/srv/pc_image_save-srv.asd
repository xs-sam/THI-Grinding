
(cl:in-package :asdf)

(defsystem "pc_image_save-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "savePcAndImage" :depends-on ("_package_savePcAndImage"))
    (:file "_package_savePcAndImage" :depends-on ("_package"))
  ))