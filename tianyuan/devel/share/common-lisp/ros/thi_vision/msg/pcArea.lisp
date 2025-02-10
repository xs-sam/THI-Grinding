; Auto-generated. Do not edit!


(cl:in-package thi_vision-msg)


;//! \htmlinclude pcArea.msg.html

(cl:defclass <pcArea> (roslisp-msg-protocol:ros-message)
  ((listPcPoint
    :reader listPcPoint
    :initarg :listPcPoint
    :type (cl:vector thi_vision-msg:position)
   :initform (cl:make-array 0 :element-type 'thi_vision-msg:position :initial-element (cl:make-instance 'thi_vision-msg:position))))
)

(cl:defclass pcArea (<pcArea>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <pcArea>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'pcArea)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name thi_vision-msg:<pcArea> is deprecated: use thi_vision-msg:pcArea instead.")))

(cl:ensure-generic-function 'listPcPoint-val :lambda-list '(m))
(cl:defmethod listPcPoint-val ((m <pcArea>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader thi_vision-msg:listPcPoint-val is deprecated.  Use thi_vision-msg:listPcPoint instead.")
  (listPcPoint m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <pcArea>) ostream)
  "Serializes a message object of type '<pcArea>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'listPcPoint))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'listPcPoint))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <pcArea>) istream)
  "Deserializes a message object of type '<pcArea>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'listPcPoint) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'listPcPoint)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'thi_vision-msg:position))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<pcArea>)))
  "Returns string type for a message object of type '<pcArea>"
  "thi_vision/pcArea")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'pcArea)))
  "Returns string type for a message object of type 'pcArea"
  "thi_vision/pcArea")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<pcArea>)))
  "Returns md5sum for a message object of type '<pcArea>"
  "1910f595f1099a0b020344db77cf9645")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'pcArea)))
  "Returns md5sum for a message object of type 'pcArea"
  "1910f595f1099a0b020344db77cf9645")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<pcArea>)))
  "Returns full string definition for message of type '<pcArea>"
  (cl:format cl:nil "position[] listPcPoint~%~%================================================================================~%MSG: thi_vision/position~%float32 x~%float32 y~%float32 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'pcArea)))
  "Returns full string definition for message of type 'pcArea"
  (cl:format cl:nil "position[] listPcPoint~%~%================================================================================~%MSG: thi_vision/position~%float32 x~%float32 y~%float32 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <pcArea>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'listPcPoint) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <pcArea>))
  "Converts a ROS message object to a list"
  (cl:list 'pcArea
    (cl:cons ':listPcPoint (listPcPoint msg))
))
