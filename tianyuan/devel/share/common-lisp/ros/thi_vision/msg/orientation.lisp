; Auto-generated. Do not edit!


(cl:in-package thi_vision-msg)


;//! \htmlinclude orientation.msg.html

(cl:defclass <orientation> (roslisp-msg-protocol:ros-message)
  ((a
    :reader a
    :initarg :a
    :type cl:float
    :initform 0.0)
   (b
    :reader b
    :initarg :b
    :type cl:float
    :initform 0.0)
   (c
    :reader c
    :initarg :c
    :type cl:float
    :initform 0.0))
)

(cl:defclass orientation (<orientation>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <orientation>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'orientation)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name thi_vision-msg:<orientation> is deprecated: use thi_vision-msg:orientation instead.")))

(cl:ensure-generic-function 'a-val :lambda-list '(m))
(cl:defmethod a-val ((m <orientation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader thi_vision-msg:a-val is deprecated.  Use thi_vision-msg:a instead.")
  (a m))

(cl:ensure-generic-function 'b-val :lambda-list '(m))
(cl:defmethod b-val ((m <orientation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader thi_vision-msg:b-val is deprecated.  Use thi_vision-msg:b instead.")
  (b m))

(cl:ensure-generic-function 'c-val :lambda-list '(m))
(cl:defmethod c-val ((m <orientation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader thi_vision-msg:c-val is deprecated.  Use thi_vision-msg:c instead.")
  (c m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <orientation>) ostream)
  "Serializes a message object of type '<orientation>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'a))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'b))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'c))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <orientation>) istream)
  "Deserializes a message object of type '<orientation>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'a) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'b) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'c) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<orientation>)))
  "Returns string type for a message object of type '<orientation>"
  "thi_vision/orientation")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'orientation)))
  "Returns string type for a message object of type 'orientation"
  "thi_vision/orientation")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<orientation>)))
  "Returns md5sum for a message object of type '<orientation>"
  "d20f63a7e99cd5689c2dcf93cf2f8085")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'orientation)))
  "Returns md5sum for a message object of type 'orientation"
  "d20f63a7e99cd5689c2dcf93cf2f8085")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<orientation>)))
  "Returns full string definition for message of type '<orientation>"
  (cl:format cl:nil "float32 a~%float32 b~%float32 c~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'orientation)))
  "Returns full string definition for message of type 'orientation"
  (cl:format cl:nil "float32 a~%float32 b~%float32 c~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <orientation>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <orientation>))
  "Converts a ROS message object to a list"
  (cl:list 'orientation
    (cl:cons ':a (a msg))
    (cl:cons ':b (b msg))
    (cl:cons ':c (c msg))
))
