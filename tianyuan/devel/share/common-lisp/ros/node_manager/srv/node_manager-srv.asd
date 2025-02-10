
(cl:in-package :asdf)

(defsystem "node_manager-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "nodeInfo" :depends-on ("_package_nodeInfo"))
    (:file "_package_nodeInfo" :depends-on ("_package"))
  ))