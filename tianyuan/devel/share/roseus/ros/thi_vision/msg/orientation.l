;; Auto-generated. Do not edit!


(when (boundp 'thi_vision::orientation)
  (if (not (find-package "THI_VISION"))
    (make-package "THI_VISION"))
  (shadow 'orientation (find-package "THI_VISION")))
(unless (find-package "THI_VISION::ORIENTATION")
  (make-package "THI_VISION::ORIENTATION"))

(in-package "ROS")
;;//! \htmlinclude orientation.msg.html


(defclass thi_vision::orientation
  :super ros::object
  :slots (_a _b _c ))

(defmethod thi_vision::orientation
  (:init
   (&key
    ((:a __a) 0.0)
    ((:b __b) 0.0)
    ((:c __c) 0.0)
    )
   (send-super :init)
   (setq _a (float __a))
   (setq _b (float __b))
   (setq _c (float __c))
   self)
  (:a
   (&optional __a)
   (if __a (setq _a __a)) _a)
  (:b
   (&optional __b)
   (if __b (setq _b __b)) _b)
  (:c
   (&optional __c)
   (if __c (setq _c __c)) _c)
  (:serialization-length
   ()
   (+
    ;; float32 _a
    4
    ;; float32 _b
    4
    ;; float32 _c
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _a
       (sys::poke _a (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _b
       (sys::poke _b (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _c
       (sys::poke _c (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _a
     (setq _a (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _b
     (setq _b (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _c
     (setq _c (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get thi_vision::orientation :md5sum-) "d20f63a7e99cd5689c2dcf93cf2f8085")
(setf (get thi_vision::orientation :datatype-) "thi_vision/orientation")
(setf (get thi_vision::orientation :definition-)
      "float32 a
float32 b
float32 c

")



(provide :thi_vision/orientation "d20f63a7e99cd5689c2dcf93cf2f8085")


