;; Auto-generated. Do not edit!


(when (boundp 'thi_vision::pcArea)
  (if (not (find-package "THI_VISION"))
    (make-package "THI_VISION"))
  (shadow 'pcArea (find-package "THI_VISION")))
(unless (find-package "THI_VISION::PCAREA")
  (make-package "THI_VISION::PCAREA"))

(in-package "ROS")
;;//! \htmlinclude pcArea.msg.html


(defclass thi_vision::pcArea
  :super ros::object
  :slots (_listPcPoint ))

(defmethod thi_vision::pcArea
  (:init
   (&key
    ((:listPcPoint __listPcPoint) ())
    )
   (send-super :init)
   (setq _listPcPoint __listPcPoint)
   self)
  (:listPcPoint
   (&rest __listPcPoint)
   (if (keywordp (car __listPcPoint))
       (send* _listPcPoint __listPcPoint)
     (progn
       (if __listPcPoint (setq _listPcPoint (car __listPcPoint)))
       _listPcPoint)))
  (:serialization-length
   ()
   (+
    ;; thi_vision/position[] _listPcPoint
    (apply #'+ (send-all _listPcPoint :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; thi_vision/position[] _listPcPoint
     (write-long (length _listPcPoint) s)
     (dolist (elem _listPcPoint)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; thi_vision/position[] _listPcPoint
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _listPcPoint (let (r) (dotimes (i n) (push (instance thi_vision::position :init) r)) r))
     (dolist (elem- _listPcPoint)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get thi_vision::pcArea :md5sum-) "1910f595f1099a0b020344db77cf9645")
(setf (get thi_vision::pcArea :datatype-) "thi_vision/pcArea")
(setf (get thi_vision::pcArea :definition-)
      "position[] listPcPoint

================================================================================
MSG: thi_vision/position
float32 x
float32 y
float32 z

")



(provide :thi_vision/pcArea "1910f595f1099a0b020344db77cf9645")


