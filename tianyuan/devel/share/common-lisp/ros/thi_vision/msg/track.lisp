; Auto-generated. Do not edit!


(cl:in-package thi_vision-msg)


;//! \htmlinclude track.msg.html

(cl:defclass <track> (roslisp-msg-protocol:ros-message)
  ((track
    :reader track
    :initarg :track
    :type (cl:vector thi_vision-msg:pose)
   :initform (cl:make-array 0 :element-type 'thi_vision-msg:pose :initial-element (cl:make-instance 'thi_vision-msg:pose))))
)

(cl:defclass track (<track>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <track>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'track)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name thi_vision-msg:<track> is deprecated: use thi_vision-msg:track instead.")))

(cl:ensure-generic-function 'track-val :lambda-list '(m))
(cl:defmethod track-val ((m <track>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader thi_vision-msg:track-val is deprecated.  Use thi_vision-msg:track instead.")
  (track m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <track>) ostream)
  "Serializes a message object of type '<track>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'track))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'track))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <track>) istream)
  "Deserializes a message object of type '<track>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'track) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'track)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'thi_vision-msg:pose))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<track>)))
  "Returns string type for a message object of type '<track>"
  "thi_vision/track")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'track)))
  "Returns string type for a message object of type 'track"
  "thi_vision/track")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<track>)))
  "Returns md5sum for a message object of type '<track>"
  "7c10583178550886efa4f5156f56c1e6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'track)))
  "Returns md5sum for a message object of type 'track"
  "7c10583178550886efa4f5156f56c1e6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<track>)))
  "Returns full string definition for message of type '<track>"
  (cl:format cl:nil "pose[] track~%~%================================================================================~%MSG: thi_vision/pose~%position point~%orientation angle~%~%================================================================================~%MSG: thi_vision/position~%float32 x~%float32 y~%float32 z~%~%================================================================================~%MSG: thi_vision/orientation~%float32 a~%float32 b~%float32 c~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'track)))
  "Returns full string definition for message of type 'track"
  (cl:format cl:nil "pose[] track~%~%================================================================================~%MSG: thi_vision/pose~%position point~%orientation angle~%~%================================================================================~%MSG: thi_vision/position~%float32 x~%float32 y~%float32 z~%~%================================================================================~%MSG: thi_vision/orientation~%float32 a~%float32 b~%float32 c~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <track>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'track) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <track>))
  "Converts a ROS message object to a list"
  (cl:list 'track
    (cl:cons ':track (track msg))
))
