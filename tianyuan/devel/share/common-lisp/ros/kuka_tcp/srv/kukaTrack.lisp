; Auto-generated. Do not edit!


(cl:in-package kuka_tcp-srv)


;//! \htmlinclude kukaTrack-request.msg.html

(cl:defclass <kukaTrack-request> (roslisp-msg-protocol:ros-message)
  ((track
    :reader track
    :initarg :track
    :type (cl:vector kuka_tcp-msg:kukaPoint)
   :initform (cl:make-array 0 :element-type 'kuka_tcp-msg:kukaPoint :initial-element (cl:make-instance 'kuka_tcp-msg:kukaPoint)))
   (speed
    :reader speed
    :initarg :speed
    :type cl:integer
    :initform 0)
   (mod
    :reader mod
    :initarg :mod
    :type cl:integer
    :initform 0))
)

(cl:defclass kukaTrack-request (<kukaTrack-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <kukaTrack-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'kukaTrack-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kuka_tcp-srv:<kukaTrack-request> is deprecated: use kuka_tcp-srv:kukaTrack-request instead.")))

(cl:ensure-generic-function 'track-val :lambda-list '(m))
(cl:defmethod track-val ((m <kukaTrack-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kuka_tcp-srv:track-val is deprecated.  Use kuka_tcp-srv:track instead.")
  (track m))

(cl:ensure-generic-function 'speed-val :lambda-list '(m))
(cl:defmethod speed-val ((m <kukaTrack-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kuka_tcp-srv:speed-val is deprecated.  Use kuka_tcp-srv:speed instead.")
  (speed m))

(cl:ensure-generic-function 'mod-val :lambda-list '(m))
(cl:defmethod mod-val ((m <kukaTrack-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kuka_tcp-srv:mod-val is deprecated.  Use kuka_tcp-srv:mod instead.")
  (mod m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <kukaTrack-request>) ostream)
  "Serializes a message object of type '<kukaTrack-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'track))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'track))
  (cl:let* ((signed (cl:slot-value msg 'speed)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'mod)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <kukaTrack-request>) istream)
  "Deserializes a message object of type '<kukaTrack-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'track) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'track)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'kuka_tcp-msg:kukaPoint))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'speed) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'mod) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<kukaTrack-request>)))
  "Returns string type for a service object of type '<kukaTrack-request>"
  "kuka_tcp/kukaTrackRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'kukaTrack-request)))
  "Returns string type for a service object of type 'kukaTrack-request"
  "kuka_tcp/kukaTrackRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<kukaTrack-request>)))
  "Returns md5sum for a message object of type '<kukaTrack-request>"
  "542cc9503392943f62224d9b971f53d6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'kukaTrack-request)))
  "Returns md5sum for a message object of type 'kukaTrack-request"
  "542cc9503392943f62224d9b971f53d6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<kukaTrack-request>)))
  "Returns full string definition for message of type '<kukaTrack-request>"
  (cl:format cl:nil "kukaPoint[] track~%int32 speed # mm/s~%int32 mod # movel 1  movej 2~%~%================================================================================~%MSG: kuka_tcp/kukaPoint~%# 机器人点位信息~%float32 x~%float32 y~%float32 z~%float32 a~%float32 b~%float32 c~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'kukaTrack-request)))
  "Returns full string definition for message of type 'kukaTrack-request"
  (cl:format cl:nil "kukaPoint[] track~%int32 speed # mm/s~%int32 mod # movel 1  movej 2~%~%================================================================================~%MSG: kuka_tcp/kukaPoint~%# 机器人点位信息~%float32 x~%float32 y~%float32 z~%float32 a~%float32 b~%float32 c~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <kukaTrack-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'track) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <kukaTrack-request>))
  "Converts a ROS message object to a list"
  (cl:list 'kukaTrack-request
    (cl:cons ':track (track msg))
    (cl:cons ':speed (speed msg))
    (cl:cons ':mod (mod msg))
))
;//! \htmlinclude kukaTrack-response.msg.html

(cl:defclass <kukaTrack-response> (roslisp-msg-protocol:ros-message)
  ((ok
    :reader ok
    :initarg :ok
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass kukaTrack-response (<kukaTrack-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <kukaTrack-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'kukaTrack-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kuka_tcp-srv:<kukaTrack-response> is deprecated: use kuka_tcp-srv:kukaTrack-response instead.")))

(cl:ensure-generic-function 'ok-val :lambda-list '(m))
(cl:defmethod ok-val ((m <kukaTrack-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kuka_tcp-srv:ok-val is deprecated.  Use kuka_tcp-srv:ok instead.")
  (ok m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <kukaTrack-response>) ostream)
  "Serializes a message object of type '<kukaTrack-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ok) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <kukaTrack-response>) istream)
  "Deserializes a message object of type '<kukaTrack-response>"
    (cl:setf (cl:slot-value msg 'ok) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<kukaTrack-response>)))
  "Returns string type for a service object of type '<kukaTrack-response>"
  "kuka_tcp/kukaTrackResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'kukaTrack-response)))
  "Returns string type for a service object of type 'kukaTrack-response"
  "kuka_tcp/kukaTrackResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<kukaTrack-response>)))
  "Returns md5sum for a message object of type '<kukaTrack-response>"
  "542cc9503392943f62224d9b971f53d6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'kukaTrack-response)))
  "Returns md5sum for a message object of type 'kukaTrack-response"
  "542cc9503392943f62224d9b971f53d6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<kukaTrack-response>)))
  "Returns full string definition for message of type '<kukaTrack-response>"
  (cl:format cl:nil "bool ok~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'kukaTrack-response)))
  "Returns full string definition for message of type 'kukaTrack-response"
  (cl:format cl:nil "bool ok~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <kukaTrack-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <kukaTrack-response>))
  "Converts a ROS message object to a list"
  (cl:list 'kukaTrack-response
    (cl:cons ':ok (ok msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'kukaTrack)))
  'kukaTrack-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'kukaTrack)))
  'kukaTrack-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'kukaTrack)))
  "Returns string type for a service object of type '<kukaTrack>"
  "kuka_tcp/kukaTrack")