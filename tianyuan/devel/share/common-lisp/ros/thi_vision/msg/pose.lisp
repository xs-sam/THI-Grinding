; Auto-generated. Do not edit!


(cl:in-package thi_vision-msg)


;//! \htmlinclude pose.msg.html

(cl:defclass <pose> (roslisp-msg-protocol:ros-message)
  ((point
    :reader point
    :initarg :point
    :type thi_vision-msg:position
    :initform (cl:make-instance 'thi_vision-msg:position))
   (angle
    :reader angle
    :initarg :angle
    :type thi_vision-msg:orientation
    :initform (cl:make-instance 'thi_vision-msg:orientation)))
)

(cl:defclass pose (<pose>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <pose>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'pose)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name thi_vision-msg:<pose> is deprecated: use thi_vision-msg:pose instead.")))

(cl:ensure-generic-function 'point-val :lambda-list '(m))
(cl:defmethod point-val ((m <pose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader thi_vision-msg:point-val is deprecated.  Use thi_vision-msg:point instead.")
  (point m))

(cl:ensure-generic-function 'angle-val :lambda-list '(m))
(cl:defmethod angle-val ((m <pose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader thi_vision-msg:angle-val is deprecated.  Use thi_vision-msg:angle instead.")
  (angle m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <pose>) ostream)
  "Serializes a message object of type '<pose>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'point) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'angle) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <pose>) istream)
  "Deserializes a message object of type '<pose>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'point) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'angle) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<pose>)))
  "Returns string type for a message object of type '<pose>"
  "thi_vision/pose")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'pose)))
  "Returns string type for a message object of type 'pose"
  "thi_vision/pose")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<pose>)))
  "Returns md5sum for a message object of type '<pose>"
  "a5c3a4fd40f2693c968403414ea57b4f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'pose)))
  "Returns md5sum for a message object of type 'pose"
  "a5c3a4fd40f2693c968403414ea57b4f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<pose>)))
  "Returns full string definition for message of type '<pose>"
  (cl:format cl:nil "position point~%orientation angle~%~%================================================================================~%MSG: thi_vision/position~%float32 x~%float32 y~%float32 z~%~%================================================================================~%MSG: thi_vision/orientation~%float32 a~%float32 b~%float32 c~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'pose)))
  "Returns full string definition for message of type 'pose"
  (cl:format cl:nil "position point~%orientation angle~%~%================================================================================~%MSG: thi_vision/position~%float32 x~%float32 y~%float32 z~%~%================================================================================~%MSG: thi_vision/orientation~%float32 a~%float32 b~%float32 c~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <pose>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'point))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'angle))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <pose>))
  "Converts a ROS message object to a list"
  (cl:list 'pose
    (cl:cons ':point (point msg))
    (cl:cons ':angle (angle msg))
))
