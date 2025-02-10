;; Auto-generated. Do not edit!


(when (boundp 'thi_vision::track)
  (if (not (find-package "THI_VISION"))
    (make-package "THI_VISION"))
  (shadow 'track (find-package "THI_VISION")))
(unless (find-package "THI_VISION::TRACK")
  (make-package "THI_VISION::TRACK"))

(in-package "ROS")
;;//! \htmlinclude track.msg.html


(defclass thi_vision::track
  :super ros::object
  :slots (_track ))

(defmethod thi_vision::track
  (:init
   (&key
    ((:track __track) ())
    )
   (send-super :init)
   (setq _track __track)
   self)
  (:track
   (&rest __track)
   (if (keywordp (car __track))
       (send* _track __track)
     (progn
       (if __track (setq _track (car __track)))
       _track)))
  (:serialization-length
   ()
   (+
    ;; thi_vision/pose[] _track
    (apply #'+ (send-all _track :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; thi_vision/pose[] _track
     (write-long (length _track) s)
     (dolist (elem _track)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; thi_vision/pose[] _track
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _track (let (r) (dotimes (i n) (push (instance thi_vision::pose :init) r)) r))
     (dolist (elem- _track)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get thi_vision::track :md5sum-) "7c10583178550886efa4f5156f56c1e6")
(setf (get thi_vision::track :datatype-) "thi_vision/track")
(setf (get thi_vision::track :definition-)
      "pose[] track

================================================================================
MSG: thi_vision/pose
position point
orientation angle

================================================================================
MSG: thi_vision/position
float32 x
float32 y
float32 z

================================================================================
MSG: thi_vision/orientation
float32 a
float32 b
float32 c

")



(provide :thi_vision/track "7c10583178550886efa4f5156f56c1e6")


