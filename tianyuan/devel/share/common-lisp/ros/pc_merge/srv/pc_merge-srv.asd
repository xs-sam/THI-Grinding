
(cl:in-package :asdf)

(defsystem "pc_merge-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "mergePc" :depends-on ("_package_mergePc"))
    (:file "_package_mergePc" :depends-on ("_package"))
  ))