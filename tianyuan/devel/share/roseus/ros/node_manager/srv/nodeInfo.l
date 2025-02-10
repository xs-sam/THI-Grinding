;; Auto-generated. Do not edit!


(when (boundp 'node_manager::nodeInfo)
  (if (not (find-package "NODE_MANAGER"))
    (make-package "NODE_MANAGER"))
  (shadow 'nodeInfo (find-package "NODE_MANAGER")))
(unless (find-package "NODE_MANAGER::NODEINFO")
  (make-package "NODE_MANAGER::NODEINFO"))
(unless (find-package "NODE_MANAGER::NODEINFOREQUEST")
  (make-package "NODE_MANAGER::NODEINFOREQUEST"))
(unless (find-package "NODE_MANAGER::NODEINFORESPONSE")
  (make-package "NODE_MANAGER::NODEINFORESPONSE"))

(in-package "ROS")





(defclass node_manager::nodeInfoRequest
  :super ros::object
  :slots (_nodeType _packName _nodeName _nodeState ))

(defmethod node_manager::nodeInfoRequest
  (:init
   (&key
    ((:nodeType __nodeType) "")
    ((:packName __packName) "")
    ((:nodeName __nodeName) "")
    ((:nodeState __nodeState) "")
    )
   (send-super :init)
   (setq _nodeType (string __nodeType))
   (setq _packName (string __packName))
   (setq _nodeName (string __nodeName))
   (setq _nodeState (string __nodeState))
   self)
  (:nodeType
   (&optional __nodeType)
   (if __nodeType (setq _nodeType __nodeType)) _nodeType)
  (:packName
   (&optional __packName)
   (if __packName (setq _packName __packName)) _packName)
  (:nodeName
   (&optional __nodeName)
   (if __nodeName (setq _nodeName __nodeName)) _nodeName)
  (:nodeState
   (&optional __nodeState)
   (if __nodeState (setq _nodeState __nodeState)) _nodeState)
  (:serialization-length
   ()
   (+
    ;; string _nodeType
    4 (length _nodeType)
    ;; string _packName
    4 (length _packName)
    ;; string _nodeName
    4 (length _nodeName)
    ;; string _nodeState
    4 (length _nodeState)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _nodeType
       (write-long (length _nodeType) s) (princ _nodeType s)
     ;; string _packName
       (write-long (length _packName) s) (princ _packName s)
     ;; string _nodeName
       (write-long (length _nodeName) s) (princ _nodeName s)
     ;; string _nodeState
       (write-long (length _nodeState) s) (princ _nodeState s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _nodeType
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _nodeType (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _packName
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _packName (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _nodeName
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _nodeName (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _nodeState
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _nodeState (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass node_manager::nodeInfoResponse
  :super ros::object
  :slots (_ok ))

(defmethod node_manager::nodeInfoResponse
  (:init
   (&key
    ((:ok __ok) nil)
    )
   (send-super :init)
   (setq _ok __ok)
   self)
  (:ok
   (&optional (__ok :null))
   (if (not (eq __ok :null)) (setq _ok __ok)) _ok)
  (:serialization-length
   ()
   (+
    ;; bool _ok
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _ok
       (if _ok (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _ok
     (setq _ok (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass node_manager::nodeInfo
  :super ros::object
  :slots ())

(setf (get node_manager::nodeInfo :md5sum-) "7752f5a8c2441ad5def7b480e5f4374d")
(setf (get node_manager::nodeInfo :datatype-) "node_manager/nodeInfo")
(setf (get node_manager::nodeInfo :request) node_manager::nodeInfoRequest)
(setf (get node_manager::nodeInfo :response) node_manager::nodeInfoResponse)

(defmethod node_manager::nodeInfoRequest
  (:response () (instance node_manager::nodeInfoResponse :init)))

(setf (get node_manager::nodeInfoRequest :md5sum-) "7752f5a8c2441ad5def7b480e5f4374d")
(setf (get node_manager::nodeInfoRequest :datatype-) "node_manager/nodeInfoRequest")
(setf (get node_manager::nodeInfoRequest :definition-)
      "string nodeType # launch / node
string packName # 包名
string nodeName # .launch名字 / 节点名字
string nodeState # start / stop
---
bool ok

")

(setf (get node_manager::nodeInfoResponse :md5sum-) "7752f5a8c2441ad5def7b480e5f4374d")
(setf (get node_manager::nodeInfoResponse :datatype-) "node_manager/nodeInfoResponse")
(setf (get node_manager::nodeInfoResponse :definition-)
      "string nodeType # launch / node
string packName # 包名
string nodeName # .launch名字 / 节点名字
string nodeState # start / stop
---
bool ok

")



(provide :node_manager/nodeInfo "7752f5a8c2441ad5def7b480e5f4374d")


