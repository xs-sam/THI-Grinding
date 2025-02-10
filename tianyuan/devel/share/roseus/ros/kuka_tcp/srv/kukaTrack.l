;; Auto-generated. Do not edit!


(when (boundp 'kuka_tcp::kukaTrack)
  (if (not (find-package "KUKA_TCP"))
    (make-package "KUKA_TCP"))
  (shadow 'kukaTrack (find-package "KUKA_TCP")))
(unless (find-package "KUKA_TCP::KUKATRACK")
  (make-package "KUKA_TCP::KUKATRACK"))
(unless (find-package "KUKA_TCP::KUKATRACKREQUEST")
  (make-package "KUKA_TCP::KUKATRACKREQUEST"))
(unless (find-package "KUKA_TCP::KUKATRACKRESPONSE")
  (make-package "KUKA_TCP::KUKATRACKRESPONSE"))

(in-package "ROS")





(defclass kuka_tcp::kukaTrackRequest
  :super ros::object
  :slots (_track _speed _mod ))

(defmethod kuka_tcp::kukaTrackRequest
  (:init
   (&key
    ((:track __track) ())
    ((:speed __speed) 0)
    ((:mod __mod) 0)
    )
   (send-super :init)
   (setq _track __track)
   (setq _speed (round __speed))
   (setq _mod (round __mod))
   self)
  (:track
   (&rest __track)
   (if (keywordp (car __track))
       (send* _track __track)
     (progn
       (if __track (setq _track (car __track)))
       _track)))
  (:speed
   (&optional __speed)
   (if __speed (setq _speed __speed)) _speed)
  (:mod
   (&optional __mod)
   (if __mod (setq _mod __mod)) _mod)
  (:serialization-length
   ()
   (+
    ;; kuka_tcp/kukaPoint[] _track
    (apply #'+ (send-all _track :serialization-length)) 4
    ;; int32 _speed
    4
    ;; int32 _mod
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kuka_tcp/kukaPoint[] _track
     (write-long (length _track) s)
     (dolist (elem _track)
       (send elem :serialize s)
       )
     ;; int32 _speed
       (write-long _speed s)
     ;; int32 _mod
       (write-long _mod s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kuka_tcp/kukaPoint[] _track
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _track (let (r) (dotimes (i n) (push (instance kuka_tcp::kukaPoint :init) r)) r))
     (dolist (elem- _track)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; int32 _speed
     (setq _speed (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _mod
     (setq _mod (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass kuka_tcp::kukaTrackResponse
  :super ros::object
  :slots (_ok ))

(defmethod kuka_tcp::kukaTrackResponse
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

(defclass kuka_tcp::kukaTrack
  :super ros::object
  :slots ())

(setf (get kuka_tcp::kukaTrack :md5sum-) "542cc9503392943f62224d9b971f53d6")
(setf (get kuka_tcp::kukaTrack :datatype-) "kuka_tcp/kukaTrack")
(setf (get kuka_tcp::kukaTrack :request) kuka_tcp::kukaTrackRequest)
(setf (get kuka_tcp::kukaTrack :response) kuka_tcp::kukaTrackResponse)

(defmethod kuka_tcp::kukaTrackRequest
  (:response () (instance kuka_tcp::kukaTrackResponse :init)))

(setf (get kuka_tcp::kukaTrackRequest :md5sum-) "542cc9503392943f62224d9b971f53d6")
(setf (get kuka_tcp::kukaTrackRequest :datatype-) "kuka_tcp/kukaTrackRequest")
(setf (get kuka_tcp::kukaTrackRequest :definition-)
      "kukaPoint[] track
int32 speed # mm/s
int32 mod # movel 1  movej 2

================================================================================
MSG: kuka_tcp/kukaPoint
# 机器人点位信息
float32 x
float32 y
float32 z
float32 a
float32 b
float32 c
---
bool ok

")

(setf (get kuka_tcp::kukaTrackResponse :md5sum-) "542cc9503392943f62224d9b971f53d6")
(setf (get kuka_tcp::kukaTrackResponse :datatype-) "kuka_tcp/kukaTrackResponse")
(setf (get kuka_tcp::kukaTrackResponse :definition-)
      "kukaPoint[] track
int32 speed # mm/s
int32 mod # movel 1  movej 2

================================================================================
MSG: kuka_tcp/kukaPoint
# 机器人点位信息
float32 x
float32 y
float32 z
float32 a
float32 b
float32 c
---
bool ok

")



(provide :kuka_tcp/kukaTrack "542cc9503392943f62224d9b971f53d6")


