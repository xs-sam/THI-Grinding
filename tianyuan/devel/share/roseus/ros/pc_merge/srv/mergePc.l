;; Auto-generated. Do not edit!


(when (boundp 'pc_merge::mergePc)
  (if (not (find-package "PC_MERGE"))
    (make-package "PC_MERGE"))
  (shadow 'mergePc (find-package "PC_MERGE")))
(unless (find-package "PC_MERGE::MERGEPC")
  (make-package "PC_MERGE::MERGEPC"))
(unless (find-package "PC_MERGE::MERGEPCREQUEST")
  (make-package "PC_MERGE::MERGEPCREQUEST"))
(unless (find-package "PC_MERGE::MERGEPCRESPONSE")
  (make-package "PC_MERGE::MERGEPCRESPONSE"))

(in-package "ROS")





(defclass pc_merge::mergePcRequest
  :super ros::object
  :slots (_workpiece _pcnum _saveName ))

(defmethod pc_merge::mergePcRequest
  (:init
   (&key
    ((:workpiece __workpiece) 0)
    ((:pcnum __pcnum) 0)
    ((:saveName __saveName) "")
    )
   (send-super :init)
   (setq _workpiece (round __workpiece))
   (setq _pcnum (round __pcnum))
   (setq _saveName (string __saveName))
   self)
  (:workpiece
   (&optional __workpiece)
   (if __workpiece (setq _workpiece __workpiece)) _workpiece)
  (:pcnum
   (&optional __pcnum)
   (if __pcnum (setq _pcnum __pcnum)) _pcnum)
  (:saveName
   (&optional __saveName)
   (if __saveName (setq _saveName __saveName)) _saveName)
  (:serialization-length
   ()
   (+
    ;; int32 _workpiece
    4
    ;; int32 _pcnum
    4
    ;; string _saveName
    4 (length _saveName)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _workpiece
       (write-long _workpiece s)
     ;; int32 _pcnum
       (write-long _pcnum s)
     ;; string _saveName
       (write-long (length _saveName) s) (princ _saveName s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _workpiece
     (setq _workpiece (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _pcnum
     (setq _pcnum (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string _saveName
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _saveName (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass pc_merge::mergePcResponse
  :super ros::object
  :slots (_ok ))

(defmethod pc_merge::mergePcResponse
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

(defclass pc_merge::mergePc
  :super ros::object
  :slots ())

(setf (get pc_merge::mergePc :md5sum-) "de99575f7cba9c0362917ec861ff5f7b")
(setf (get pc_merge::mergePc :datatype-) "pc_merge/mergePc")
(setf (get pc_merge::mergePc :request) pc_merge::mergePcRequest)
(setf (get pc_merge::mergePc :response) pc_merge::mergePcResponse)

(defmethod pc_merge::mergePcRequest
  (:response () (instance pc_merge::mergePcResponse :init)))

(setf (get pc_merge::mergePcRequest :md5sum-) "de99575f7cba9c0362917ec861ff5f7b")
(setf (get pc_merge::mergePcRequest :datatype-) "pc_merge/mergePcRequest")
(setf (get pc_merge::mergePcRequest :definition-)
      "int32 workpiece # 0是上半件 1是下右半件 2是下左半件
int32 pcnum # 需要合成的点云数量 如果为-1 则采用参数文件中的量
string saveName # 保存的文件名 为空则用参数文件中的值
---
bool ok

")

(setf (get pc_merge::mergePcResponse :md5sum-) "de99575f7cba9c0362917ec861ff5f7b")
(setf (get pc_merge::mergePcResponse :datatype-) "pc_merge/mergePcResponse")
(setf (get pc_merge::mergePcResponse :definition-)
      "int32 workpiece # 0是上半件 1是下右半件 2是下左半件
int32 pcnum # 需要合成的点云数量 如果为-1 则采用参数文件中的量
string saveName # 保存的文件名 为空则用参数文件中的值
---
bool ok

")



(provide :pc_merge/mergePc "de99575f7cba9c0362917ec861ff5f7b")


