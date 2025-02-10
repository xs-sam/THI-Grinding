; Auto-generated. Do not edit!


(cl:in-package thi_vision-srv)


;//! \htmlinclude visionTracks-request.msg.html

(cl:defclass <visionTracks-request> (roslisp-msg-protocol:ros-message)
  ((pointCloudFile
    :reader pointCloudFile
    :initarg :pointCloudFile
    :type cl:string
    :initform "")
   (fileType
    :reader fileType
    :initarg :fileType
    :type cl:string
    :initform "")
   (workpiece
    :reader workpiece
    :initarg :workpiece
    :type cl:integer
    :initform 0)
   (press
    :reader press
    :initarg :press
    :type cl:float
    :initform 0.0)
   (width
    :reader width
    :initarg :width
    :type cl:float
    :initform 0.0)
   (grooveWidth
    :reader grooveWidth
    :initarg :grooveWidth
    :type cl:float
    :initform 0.0)
   (avoidLeft
    :reader avoidLeft
    :initarg :avoidLeft
    :type cl:float
    :initform 0.0)
   (avoidRight
    :reader avoidRight
    :initarg :avoidRight
    :type cl:float
    :initform 0.0)
   (length
    :reader length
    :initarg :length
    :type cl:float
    :initform 0.0)
   (angleLeft
    :reader angleLeft
    :initarg :angleLeft
    :type thi_vision-msg:orientation
    :initform (cl:make-instance 'thi_vision-msg:orientation))
   (angleRight
    :reader angleRight
    :initarg :angleRight
    :type thi_vision-msg:orientation
    :initform (cl:make-instance 'thi_vision-msg:orientation))
   (listPcAreas
    :reader listPcAreas
    :initarg :listPcAreas
    :type (cl:vector thi_vision-msg:pcArea)
   :initform (cl:make-array 0 :element-type 'thi_vision-msg:pcArea :initial-element (cl:make-instance 'thi_vision-msg:pcArea))))
)

(cl:defclass visionTracks-request (<visionTracks-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <visionTracks-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'visionTracks-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name thi_vision-srv:<visionTracks-request> is deprecated: use thi_vision-srv:visionTracks-request instead.")))

(cl:ensure-generic-function 'pointCloudFile-val :lambda-list '(m))
(cl:defmethod pointCloudFile-val ((m <visionTracks-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader thi_vision-srv:pointCloudFile-val is deprecated.  Use thi_vision-srv:pointCloudFile instead.")
  (pointCloudFile m))

(cl:ensure-generic-function 'fileType-val :lambda-list '(m))
(cl:defmethod fileType-val ((m <visionTracks-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader thi_vision-srv:fileType-val is deprecated.  Use thi_vision-srv:fileType instead.")
  (fileType m))

(cl:ensure-generic-function 'workpiece-val :lambda-list '(m))
(cl:defmethod workpiece-val ((m <visionTracks-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader thi_vision-srv:workpiece-val is deprecated.  Use thi_vision-srv:workpiece instead.")
  (workpiece m))

(cl:ensure-generic-function 'press-val :lambda-list '(m))
(cl:defmethod press-val ((m <visionTracks-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader thi_vision-srv:press-val is deprecated.  Use thi_vision-srv:press instead.")
  (press m))

(cl:ensure-generic-function 'width-val :lambda-list '(m))
(cl:defmethod width-val ((m <visionTracks-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader thi_vision-srv:width-val is deprecated.  Use thi_vision-srv:width instead.")
  (width m))

(cl:ensure-generic-function 'grooveWidth-val :lambda-list '(m))
(cl:defmethod grooveWidth-val ((m <visionTracks-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader thi_vision-srv:grooveWidth-val is deprecated.  Use thi_vision-srv:grooveWidth instead.")
  (grooveWidth m))

(cl:ensure-generic-function 'avoidLeft-val :lambda-list '(m))
(cl:defmethod avoidLeft-val ((m <visionTracks-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader thi_vision-srv:avoidLeft-val is deprecated.  Use thi_vision-srv:avoidLeft instead.")
  (avoidLeft m))

(cl:ensure-generic-function 'avoidRight-val :lambda-list '(m))
(cl:defmethod avoidRight-val ((m <visionTracks-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader thi_vision-srv:avoidRight-val is deprecated.  Use thi_vision-srv:avoidRight instead.")
  (avoidRight m))

(cl:ensure-generic-function 'length-val :lambda-list '(m))
(cl:defmethod length-val ((m <visionTracks-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader thi_vision-srv:length-val is deprecated.  Use thi_vision-srv:length instead.")
  (length m))

(cl:ensure-generic-function 'angleLeft-val :lambda-list '(m))
(cl:defmethod angleLeft-val ((m <visionTracks-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader thi_vision-srv:angleLeft-val is deprecated.  Use thi_vision-srv:angleLeft instead.")
  (angleLeft m))

(cl:ensure-generic-function 'angleRight-val :lambda-list '(m))
(cl:defmethod angleRight-val ((m <visionTracks-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader thi_vision-srv:angleRight-val is deprecated.  Use thi_vision-srv:angleRight instead.")
  (angleRight m))

(cl:ensure-generic-function 'listPcAreas-val :lambda-list '(m))
(cl:defmethod listPcAreas-val ((m <visionTracks-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader thi_vision-srv:listPcAreas-val is deprecated.  Use thi_vision-srv:listPcAreas instead.")
  (listPcAreas m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <visionTracks-request>) ostream)
  "Serializes a message object of type '<visionTracks-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'pointCloudFile))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'pointCloudFile))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'fileType))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'fileType))
  (cl:let* ((signed (cl:slot-value msg 'workpiece)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'press))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'width))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'grooveWidth))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'avoidLeft))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'avoidRight))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'length))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'angleLeft) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'angleRight) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'listPcAreas))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'listPcAreas))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <visionTracks-request>) istream)
  "Deserializes a message object of type '<visionTracks-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'pointCloudFile) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'pointCloudFile) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'fileType) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'fileType) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'workpiece) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'press) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'width) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'grooveWidth) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'avoidLeft) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'avoidRight) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'length) (roslisp-utils:decode-single-float-bits bits)))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'angleLeft) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'angleRight) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'listPcAreas) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'listPcAreas)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'thi_vision-msg:pcArea))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<visionTracks-request>)))
  "Returns string type for a service object of type '<visionTracks-request>"
  "thi_vision/visionTracksRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'visionTracks-request)))
  "Returns string type for a service object of type 'visionTracks-request"
  "thi_vision/visionTracksRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<visionTracks-request>)))
  "Returns md5sum for a message object of type '<visionTracks-request>"
  "54640a39f52ffd596022a5e1d0dc6ffe")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'visionTracks-request)))
  "Returns md5sum for a message object of type 'visionTracks-request"
  "54640a39f52ffd596022a5e1d0dc6ffe")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<visionTracks-request>)))
  "Returns full string definition for message of type '<visionTracks-request>"
  (cl:format cl:nil "string pointCloudFile~%string fileType     # \"ply\" \"pcd\" \"txt\"~%int32  workpiece    #工件号 0:上半间 1:下半件 右面~%float32 press       #下压量~%float32 width       #道宽 单位mm~%float32 grooveWidth #预留的槽宽~%float32 avoidLeft   #x轴正方向左侧避障弧长~%float32 avoidRight  #x轴正方向右侧避障弧长~%float32 length      #可打磨的最长区域~%orientation angleLeft  #x轴正方向左侧避障固定姿态角度(a,b,c对应法兰盘rpy)~%orientation angleRight #x轴正方向右侧避障固定姿态角度~%pcArea[] listPcAreas~%~%================================================================================~%MSG: thi_vision/orientation~%float32 a~%float32 b~%float32 c~%~%================================================================================~%MSG: thi_vision/pcArea~%position[] listPcPoint~%~%================================================================================~%MSG: thi_vision/position~%float32 x~%float32 y~%float32 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'visionTracks-request)))
  "Returns full string definition for message of type 'visionTracks-request"
  (cl:format cl:nil "string pointCloudFile~%string fileType     # \"ply\" \"pcd\" \"txt\"~%int32  workpiece    #工件号 0:上半间 1:下半件 右面~%float32 press       #下压量~%float32 width       #道宽 单位mm~%float32 grooveWidth #预留的槽宽~%float32 avoidLeft   #x轴正方向左侧避障弧长~%float32 avoidRight  #x轴正方向右侧避障弧长~%float32 length      #可打磨的最长区域~%orientation angleLeft  #x轴正方向左侧避障固定姿态角度(a,b,c对应法兰盘rpy)~%orientation angleRight #x轴正方向右侧避障固定姿态角度~%pcArea[] listPcAreas~%~%================================================================================~%MSG: thi_vision/orientation~%float32 a~%float32 b~%float32 c~%~%================================================================================~%MSG: thi_vision/pcArea~%position[] listPcPoint~%~%================================================================================~%MSG: thi_vision/position~%float32 x~%float32 y~%float32 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <visionTracks-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'pointCloudFile))
     4 (cl:length (cl:slot-value msg 'fileType))
     4
     4
     4
     4
     4
     4
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'angleLeft))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'angleRight))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'listPcAreas) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <visionTracks-request>))
  "Converts a ROS message object to a list"
  (cl:list 'visionTracks-request
    (cl:cons ':pointCloudFile (pointCloudFile msg))
    (cl:cons ':fileType (fileType msg))
    (cl:cons ':workpiece (workpiece msg))
    (cl:cons ':press (press msg))
    (cl:cons ':width (width msg))
    (cl:cons ':grooveWidth (grooveWidth msg))
    (cl:cons ':avoidLeft (avoidLeft msg))
    (cl:cons ':avoidRight (avoidRight msg))
    (cl:cons ':length (length msg))
    (cl:cons ':angleLeft (angleLeft msg))
    (cl:cons ':angleRight (angleRight msg))
    (cl:cons ':listPcAreas (listPcAreas msg))
))
;//! \htmlinclude visionTracks-response.msg.html

(cl:defclass <visionTracks-response> (roslisp-msg-protocol:ros-message)
  ((tracks
    :reader tracks
    :initarg :tracks
    :type (cl:vector thi_vision-msg:track)
   :initform (cl:make-array 0 :element-type 'thi_vision-msg:track :initial-element (cl:make-instance 'thi_vision-msg:track)))
   (ok
    :reader ok
    :initarg :ok
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass visionTracks-response (<visionTracks-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <visionTracks-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'visionTracks-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name thi_vision-srv:<visionTracks-response> is deprecated: use thi_vision-srv:visionTracks-response instead.")))

(cl:ensure-generic-function 'tracks-val :lambda-list '(m))
(cl:defmethod tracks-val ((m <visionTracks-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader thi_vision-srv:tracks-val is deprecated.  Use thi_vision-srv:tracks instead.")
  (tracks m))

(cl:ensure-generic-function 'ok-val :lambda-list '(m))
(cl:defmethod ok-val ((m <visionTracks-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader thi_vision-srv:ok-val is deprecated.  Use thi_vision-srv:ok instead.")
  (ok m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <visionTracks-response>) ostream)
  "Serializes a message object of type '<visionTracks-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'tracks))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'tracks))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ok) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <visionTracks-response>) istream)
  "Deserializes a message object of type '<visionTracks-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'tracks) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'tracks)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'thi_vision-msg:track))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
    (cl:setf (cl:slot-value msg 'ok) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<visionTracks-response>)))
  "Returns string type for a service object of type '<visionTracks-response>"
  "thi_vision/visionTracksResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'visionTracks-response)))
  "Returns string type for a service object of type 'visionTracks-response"
  "thi_vision/visionTracksResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<visionTracks-response>)))
  "Returns md5sum for a message object of type '<visionTracks-response>"
  "54640a39f52ffd596022a5e1d0dc6ffe")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'visionTracks-response)))
  "Returns md5sum for a message object of type 'visionTracks-response"
  "54640a39f52ffd596022a5e1d0dc6ffe")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<visionTracks-response>)))
  "Returns full string definition for message of type '<visionTracks-response>"
  (cl:format cl:nil "track[] tracks~%bool ok~%~%~%================================================================================~%MSG: thi_vision/track~%pose[] track~%~%================================================================================~%MSG: thi_vision/pose~%position point~%orientation angle~%~%================================================================================~%MSG: thi_vision/position~%float32 x~%float32 y~%float32 z~%~%================================================================================~%MSG: thi_vision/orientation~%float32 a~%float32 b~%float32 c~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'visionTracks-response)))
  "Returns full string definition for message of type 'visionTracks-response"
  (cl:format cl:nil "track[] tracks~%bool ok~%~%~%================================================================================~%MSG: thi_vision/track~%pose[] track~%~%================================================================================~%MSG: thi_vision/pose~%position point~%orientation angle~%~%================================================================================~%MSG: thi_vision/position~%float32 x~%float32 y~%float32 z~%~%================================================================================~%MSG: thi_vision/orientation~%float32 a~%float32 b~%float32 c~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <visionTracks-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'tracks) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <visionTracks-response>))
  "Converts a ROS message object to a list"
  (cl:list 'visionTracks-response
    (cl:cons ':tracks (tracks msg))
    (cl:cons ':ok (ok msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'visionTracks)))
  'visionTracks-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'visionTracks)))
  'visionTracks-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'visionTracks)))
  "Returns string type for a service object of type '<visionTracks>"
  "thi_vision/visionTracks")