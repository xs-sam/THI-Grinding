; Auto-generated. Do not edit!


(cl:in-package pc_merge-srv)


;//! \htmlinclude mergePc-request.msg.html

(cl:defclass <mergePc-request> (roslisp-msg-protocol:ros-message)
  ((workpiece
    :reader workpiece
    :initarg :workpiece
    :type cl:integer
    :initform 0)
   (pcnum
    :reader pcnum
    :initarg :pcnum
    :type cl:integer
    :initform 0)
   (saveName
    :reader saveName
    :initarg :saveName
    :type cl:string
    :initform ""))
)

(cl:defclass mergePc-request (<mergePc-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <mergePc-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'mergePc-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pc_merge-srv:<mergePc-request> is deprecated: use pc_merge-srv:mergePc-request instead.")))

(cl:ensure-generic-function 'workpiece-val :lambda-list '(m))
(cl:defmethod workpiece-val ((m <mergePc-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pc_merge-srv:workpiece-val is deprecated.  Use pc_merge-srv:workpiece instead.")
  (workpiece m))

(cl:ensure-generic-function 'pcnum-val :lambda-list '(m))
(cl:defmethod pcnum-val ((m <mergePc-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pc_merge-srv:pcnum-val is deprecated.  Use pc_merge-srv:pcnum instead.")
  (pcnum m))

(cl:ensure-generic-function 'saveName-val :lambda-list '(m))
(cl:defmethod saveName-val ((m <mergePc-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pc_merge-srv:saveName-val is deprecated.  Use pc_merge-srv:saveName instead.")
  (saveName m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <mergePc-request>) ostream)
  "Serializes a message object of type '<mergePc-request>"
  (cl:let* ((signed (cl:slot-value msg 'workpiece)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'pcnum)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'saveName))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'saveName))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <mergePc-request>) istream)
  "Deserializes a message object of type '<mergePc-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'workpiece) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'pcnum) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'saveName) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'saveName) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<mergePc-request>)))
  "Returns string type for a service object of type '<mergePc-request>"
  "pc_merge/mergePcRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'mergePc-request)))
  "Returns string type for a service object of type 'mergePc-request"
  "pc_merge/mergePcRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<mergePc-request>)))
  "Returns md5sum for a message object of type '<mergePc-request>"
  "de99575f7cba9c0362917ec861ff5f7b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'mergePc-request)))
  "Returns md5sum for a message object of type 'mergePc-request"
  "de99575f7cba9c0362917ec861ff5f7b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<mergePc-request>)))
  "Returns full string definition for message of type '<mergePc-request>"
  (cl:format cl:nil "int32 workpiece # 0是上半件 1是下右半件 2是下左半件~%int32 pcnum # 需要合成的点云数量 如果为-1 则采用参数文件中的量~%string saveName # 保存的文件名 为空则用参数文件中的值~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'mergePc-request)))
  "Returns full string definition for message of type 'mergePc-request"
  (cl:format cl:nil "int32 workpiece # 0是上半件 1是下右半件 2是下左半件~%int32 pcnum # 需要合成的点云数量 如果为-1 则采用参数文件中的量~%string saveName # 保存的文件名 为空则用参数文件中的值~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <mergePc-request>))
  (cl:+ 0
     4
     4
     4 (cl:length (cl:slot-value msg 'saveName))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <mergePc-request>))
  "Converts a ROS message object to a list"
  (cl:list 'mergePc-request
    (cl:cons ':workpiece (workpiece msg))
    (cl:cons ':pcnum (pcnum msg))
    (cl:cons ':saveName (saveName msg))
))
;//! \htmlinclude mergePc-response.msg.html

(cl:defclass <mergePc-response> (roslisp-msg-protocol:ros-message)
  ((ok
    :reader ok
    :initarg :ok
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass mergePc-response (<mergePc-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <mergePc-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'mergePc-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pc_merge-srv:<mergePc-response> is deprecated: use pc_merge-srv:mergePc-response instead.")))

(cl:ensure-generic-function 'ok-val :lambda-list '(m))
(cl:defmethod ok-val ((m <mergePc-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pc_merge-srv:ok-val is deprecated.  Use pc_merge-srv:ok instead.")
  (ok m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <mergePc-response>) ostream)
  "Serializes a message object of type '<mergePc-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ok) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <mergePc-response>) istream)
  "Deserializes a message object of type '<mergePc-response>"
    (cl:setf (cl:slot-value msg 'ok) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<mergePc-response>)))
  "Returns string type for a service object of type '<mergePc-response>"
  "pc_merge/mergePcResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'mergePc-response)))
  "Returns string type for a service object of type 'mergePc-response"
  "pc_merge/mergePcResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<mergePc-response>)))
  "Returns md5sum for a message object of type '<mergePc-response>"
  "de99575f7cba9c0362917ec861ff5f7b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'mergePc-response)))
  "Returns md5sum for a message object of type 'mergePc-response"
  "de99575f7cba9c0362917ec861ff5f7b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<mergePc-response>)))
  "Returns full string definition for message of type '<mergePc-response>"
  (cl:format cl:nil "bool ok~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'mergePc-response)))
  "Returns full string definition for message of type 'mergePc-response"
  (cl:format cl:nil "bool ok~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <mergePc-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <mergePc-response>))
  "Converts a ROS message object to a list"
  (cl:list 'mergePc-response
    (cl:cons ':ok (ok msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'mergePc)))
  'mergePc-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'mergePc)))
  'mergePc-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'mergePc)))
  "Returns string type for a service object of type '<mergePc>"
  "pc_merge/mergePc")