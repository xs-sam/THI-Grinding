;; Auto-generated. Do not edit!


(when (boundp 'thi_vision::pose)
  (if (not (find-package "THI_VISION"))
    (make-package "THI_VISION"))
  (shadow 'pose (find-package "THI_VISION")))
(unless (find-package "THI_VISION::POSE")
  (make-package "THI_VISION::POSE"))

(in-package "ROS")
;;//! \htmlinclude pose.msg.html


(defclass thi_vision::pose
  :super ros::object
  :slots (_point _angle ))

(defmethod thi_vision::pose
  (:init
   (&key
    ((:point __point) (instance thi_vision::position :init))
    ((:angle __angle) (instance thi_vision::orientation :init))
    )
   (send-super :init)
   (setq _point __point)
   (setq _angle __angle)
   self)
  (:point
   (&rest __point)
   (if (keywordp (car __point))
       (send* _point __point)
     (progn
       (if __point (setq _point (car __point)))
       _point)))
  (:angle
   (&rest __angle)
   (if (keywordp (car __angle))
       (send* _angle __angle)
     (progn
       (if __angle (setq _angle (car __angle)))
       _angle)))
  (:serialization-length
   ()
   (+
    ;; thi_vision/position _point
    (send _point :serialization-length)
    ;; thi_vision/orientation _angle
    (send _angle :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; thi_vision/position _point
       (send _point :serialize s)
     ;; thi_vision/orientation _angle
       (send _angle :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; thi_vision/position _point
     (send _point :deserialize buf ptr-) (incf ptr- (send _point :serialization-length))
   ;; thi_vision/orientation _angle
     (send _angle :deserialize buf ptr-) (incf ptr- (send _angle :serialization-length))
   ;;
   self)
  )

(setf (get thi_vision::pose :md5sum-) "a5c3a4fd40f2693c968403414ea57b4f")
(setf (get thi_vision::pose :datatype-) "thi_vision/pose")
(setf (get thi_vision::pose :definition-)
      "position point
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



(provide :thi_vision/pose "a5c3a4fd40f2693c968403414ea57b4f")


