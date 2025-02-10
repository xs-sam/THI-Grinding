;; Auto-generated. Do not edit!


(when (boundp 'thi_vision::visionTracks)
  (if (not (find-package "THI_VISION"))
    (make-package "THI_VISION"))
  (shadow 'visionTracks (find-package "THI_VISION")))
(unless (find-package "THI_VISION::VISIONTRACKS")
  (make-package "THI_VISION::VISIONTRACKS"))
(unless (find-package "THI_VISION::VISIONTRACKSREQUEST")
  (make-package "THI_VISION::VISIONTRACKSREQUEST"))
(unless (find-package "THI_VISION::VISIONTRACKSRESPONSE")
  (make-package "THI_VISION::VISIONTRACKSRESPONSE"))

(in-package "ROS")





(defclass thi_vision::visionTracksRequest
  :super ros::object
  :slots (_pointCloudFile _fileType _workpiece _press _width _grooveWidth _avoidLeft _avoidRight _length _angleLeft _angleRight _listPcAreas ))

(defmethod thi_vision::visionTracksRequest
  (:init
   (&key
    ((:pointCloudFile __pointCloudFile) "")
    ((:fileType __fileType) "")
    ((:workpiece __workpiece) 0)
    ((:press __press) 0.0)
    ((:width __width) 0.0)
    ((:grooveWidth __grooveWidth) 0.0)
    ((:avoidLeft __avoidLeft) 0.0)
    ((:avoidRight __avoidRight) 0.0)
    ((:length __length) 0.0)
    ((:angleLeft __angleLeft) (instance thi_vision::orientation :init))
    ((:angleRight __angleRight) (instance thi_vision::orientation :init))
    ((:listPcAreas __listPcAreas) ())
    )
   (send-super :init)
   (setq _pointCloudFile (string __pointCloudFile))
   (setq _fileType (string __fileType))
   (setq _workpiece (round __workpiece))
   (setq _press (float __press))
   (setq _width (float __width))
   (setq _grooveWidth (float __grooveWidth))
   (setq _avoidLeft (float __avoidLeft))
   (setq _avoidRight (float __avoidRight))
   (setq _length (float __length))
   (setq _angleLeft __angleLeft)
   (setq _angleRight __angleRight)
   (setq _listPcAreas __listPcAreas)
   self)
  (:pointCloudFile
   (&optional __pointCloudFile)
   (if __pointCloudFile (setq _pointCloudFile __pointCloudFile)) _pointCloudFile)
  (:fileType
   (&optional __fileType)
   (if __fileType (setq _fileType __fileType)) _fileType)
  (:workpiece
   (&optional __workpiece)
   (if __workpiece (setq _workpiece __workpiece)) _workpiece)
  (:press
   (&optional __press)
   (if __press (setq _press __press)) _press)
  (:width
   (&optional __width)
   (if __width (setq _width __width)) _width)
  (:grooveWidth
   (&optional __grooveWidth)
   (if __grooveWidth (setq _grooveWidth __grooveWidth)) _grooveWidth)
  (:avoidLeft
   (&optional __avoidLeft)
   (if __avoidLeft (setq _avoidLeft __avoidLeft)) _avoidLeft)
  (:avoidRight
   (&optional __avoidRight)
   (if __avoidRight (setq _avoidRight __avoidRight)) _avoidRight)
  (:length
   (&optional __length)
   (if __length (setq _length __length)) _length)
  (:angleLeft
   (&rest __angleLeft)
   (if (keywordp (car __angleLeft))
       (send* _angleLeft __angleLeft)
     (progn
       (if __angleLeft (setq _angleLeft (car __angleLeft)))
       _angleLeft)))
  (:angleRight
   (&rest __angleRight)
   (if (keywordp (car __angleRight))
       (send* _angleRight __angleRight)
     (progn
       (if __angleRight (setq _angleRight (car __angleRight)))
       _angleRight)))
  (:listPcAreas
   (&rest __listPcAreas)
   (if (keywordp (car __listPcAreas))
       (send* _listPcAreas __listPcAreas)
     (progn
       (if __listPcAreas (setq _listPcAreas (car __listPcAreas)))
       _listPcAreas)))
  (:serialization-length
   ()
   (+
    ;; string _pointCloudFile
    4 (length _pointCloudFile)
    ;; string _fileType
    4 (length _fileType)
    ;; int32 _workpiece
    4
    ;; float32 _press
    4
    ;; float32 _width
    4
    ;; float32 _grooveWidth
    4
    ;; float32 _avoidLeft
    4
    ;; float32 _avoidRight
    4
    ;; float32 _length
    4
    ;; thi_vision/orientation _angleLeft
    (send _angleLeft :serialization-length)
    ;; thi_vision/orientation _angleRight
    (send _angleRight :serialization-length)
    ;; thi_vision/pcArea[] _listPcAreas
    (apply #'+ (send-all _listPcAreas :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _pointCloudFile
       (write-long (length _pointCloudFile) s) (princ _pointCloudFile s)
     ;; string _fileType
       (write-long (length _fileType) s) (princ _fileType s)
     ;; int32 _workpiece
       (write-long _workpiece s)
     ;; float32 _press
       (sys::poke _press (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _width
       (sys::poke _width (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _grooveWidth
       (sys::poke _grooveWidth (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _avoidLeft
       (sys::poke _avoidLeft (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _avoidRight
       (sys::poke _avoidRight (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _length
       (sys::poke _length (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; thi_vision/orientation _angleLeft
       (send _angleLeft :serialize s)
     ;; thi_vision/orientation _angleRight
       (send _angleRight :serialize s)
     ;; thi_vision/pcArea[] _listPcAreas
     (write-long (length _listPcAreas) s)
     (dolist (elem _listPcAreas)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _pointCloudFile
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _pointCloudFile (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _fileType
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _fileType (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int32 _workpiece
     (setq _workpiece (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _press
     (setq _press (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _width
     (setq _width (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _grooveWidth
     (setq _grooveWidth (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _avoidLeft
     (setq _avoidLeft (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _avoidRight
     (setq _avoidRight (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _length
     (setq _length (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; thi_vision/orientation _angleLeft
     (send _angleLeft :deserialize buf ptr-) (incf ptr- (send _angleLeft :serialization-length))
   ;; thi_vision/orientation _angleRight
     (send _angleRight :deserialize buf ptr-) (incf ptr- (send _angleRight :serialization-length))
   ;; thi_vision/pcArea[] _listPcAreas
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _listPcAreas (let (r) (dotimes (i n) (push (instance thi_vision::pcArea :init) r)) r))
     (dolist (elem- _listPcAreas)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(defclass thi_vision::visionTracksResponse
  :super ros::object
  :slots (_tracks _ok ))

(defmethod thi_vision::visionTracksResponse
  (:init
   (&key
    ((:tracks __tracks) ())
    ((:ok __ok) nil)
    )
   (send-super :init)
   (setq _tracks __tracks)
   (setq _ok __ok)
   self)
  (:tracks
   (&rest __tracks)
   (if (keywordp (car __tracks))
       (send* _tracks __tracks)
     (progn
       (if __tracks (setq _tracks (car __tracks)))
       _tracks)))
  (:ok
   (&optional (__ok :null))
   (if (not (eq __ok :null)) (setq _ok __ok)) _ok)
  (:serialization-length
   ()
   (+
    ;; thi_vision/track[] _tracks
    (apply #'+ (send-all _tracks :serialization-length)) 4
    ;; bool _ok
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; thi_vision/track[] _tracks
     (write-long (length _tracks) s)
     (dolist (elem _tracks)
       (send elem :serialize s)
       )
     ;; bool _ok
       (if _ok (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; thi_vision/track[] _tracks
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _tracks (let (r) (dotimes (i n) (push (instance thi_vision::track :init) r)) r))
     (dolist (elem- _tracks)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; bool _ok
     (setq _ok (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass thi_vision::visionTracks
  :super ros::object
  :slots ())

(setf (get thi_vision::visionTracks :md5sum-) "54640a39f52ffd596022a5e1d0dc6ffe")
(setf (get thi_vision::visionTracks :datatype-) "thi_vision/visionTracks")
(setf (get thi_vision::visionTracks :request) thi_vision::visionTracksRequest)
(setf (get thi_vision::visionTracks :response) thi_vision::visionTracksResponse)

(defmethod thi_vision::visionTracksRequest
  (:response () (instance thi_vision::visionTracksResponse :init)))

(setf (get thi_vision::visionTracksRequest :md5sum-) "54640a39f52ffd596022a5e1d0dc6ffe")
(setf (get thi_vision::visionTracksRequest :datatype-) "thi_vision/visionTracksRequest")
(setf (get thi_vision::visionTracksRequest :definition-)
      "string pointCloudFile
string fileType     # \"ply\" \"pcd\" \"txt\"
int32  workpiece    #工件号 0:上半间 1:下半件 右面
float32 press       #下压量
float32 width       #道宽 单位mm
float32 grooveWidth #预留的槽宽
float32 avoidLeft   #x轴正方向左侧避障弧长
float32 avoidRight  #x轴正方向右侧避障弧长
float32 length      #可打磨的最长区域
orientation angleLeft  #x轴正方向左侧避障固定姿态角度(a,b,c对应法兰盘rpy)
orientation angleRight #x轴正方向右侧避障固定姿态角度
pcArea[] listPcAreas

================================================================================
MSG: thi_vision/orientation
float32 a
float32 b
float32 c

================================================================================
MSG: thi_vision/pcArea
position[] listPcPoint

================================================================================
MSG: thi_vision/position
float32 x
float32 y
float32 z
---
track[] tracks
bool ok


================================================================================
MSG: thi_vision/track
pose[] track

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

(setf (get thi_vision::visionTracksResponse :md5sum-) "54640a39f52ffd596022a5e1d0dc6ffe")
(setf (get thi_vision::visionTracksResponse :datatype-) "thi_vision/visionTracksResponse")
(setf (get thi_vision::visionTracksResponse :definition-)
      "string pointCloudFile
string fileType     # \"ply\" \"pcd\" \"txt\"
int32  workpiece    #工件号 0:上半间 1:下半件 右面
float32 press       #下压量
float32 width       #道宽 单位mm
float32 grooveWidth #预留的槽宽
float32 avoidLeft   #x轴正方向左侧避障弧长
float32 avoidRight  #x轴正方向右侧避障弧长
float32 length      #可打磨的最长区域
orientation angleLeft  #x轴正方向左侧避障固定姿态角度(a,b,c对应法兰盘rpy)
orientation angleRight #x轴正方向右侧避障固定姿态角度
pcArea[] listPcAreas

================================================================================
MSG: thi_vision/orientation
float32 a
float32 b
float32 c

================================================================================
MSG: thi_vision/pcArea
position[] listPcPoint

================================================================================
MSG: thi_vision/position
float32 x
float32 y
float32 z
---
track[] tracks
bool ok


================================================================================
MSG: thi_vision/track
pose[] track

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



(provide :thi_vision/visionTracks "54640a39f52ffd596022a5e1d0dc6ffe")


