; Auto-generated. Do not edit!


(cl:in-package node_manager-srv)


;//! \htmlinclude nodeInfo-request.msg.html

(cl:defclass <nodeInfo-request> (roslisp-msg-protocol:ros-message)
  ((nodeType
    :reader nodeType
    :initarg :nodeType
    :type cl:string
    :initform "")
   (packName
    :reader packName
    :initarg :packName
    :type cl:string
    :initform "")
   (nodeName
    :reader nodeName
    :initarg :nodeName
    :type cl:string
    :initform "")
   (nodeState
    :reader nodeState
    :initarg :nodeState
    :type cl:string
    :initform ""))
)

(cl:defclass nodeInfo-request (<nodeInfo-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <nodeInfo-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'nodeInfo-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name node_manager-srv:<nodeInfo-request> is deprecated: use node_manager-srv:nodeInfo-request instead.")))

(cl:ensure-generic-function 'nodeType-val :lambda-list '(m))
(cl:defmethod nodeType-val ((m <nodeInfo-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader node_manager-srv:nodeType-val is deprecated.  Use node_manager-srv:nodeType instead.")
  (nodeType m))

(cl:ensure-generic-function 'packName-val :lambda-list '(m))
(cl:defmethod packName-val ((m <nodeInfo-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader node_manager-srv:packName-val is deprecated.  Use node_manager-srv:packName instead.")
  (packName m))

(cl:ensure-generic-function 'nodeName-val :lambda-list '(m))
(cl:defmethod nodeName-val ((m <nodeInfo-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader node_manager-srv:nodeName-val is deprecated.  Use node_manager-srv:nodeName instead.")
  (nodeName m))

(cl:ensure-generic-function 'nodeState-val :lambda-list '(m))
(cl:defmethod nodeState-val ((m <nodeInfo-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader node_manager-srv:nodeState-val is deprecated.  Use node_manager-srv:nodeState instead.")
  (nodeState m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <nodeInfo-request>) ostream)
  "Serializes a message object of type '<nodeInfo-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'nodeType))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'nodeType))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'packName))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'packName))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'nodeName))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'nodeName))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'nodeState))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'nodeState))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <nodeInfo-request>) istream)
  "Deserializes a message object of type '<nodeInfo-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'nodeType) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'nodeType) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'packName) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'packName) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'nodeName) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'nodeName) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'nodeState) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'nodeState) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<nodeInfo-request>)))
  "Returns string type for a service object of type '<nodeInfo-request>"
  "node_manager/nodeInfoRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'nodeInfo-request)))
  "Returns string type for a service object of type 'nodeInfo-request"
  "node_manager/nodeInfoRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<nodeInfo-request>)))
  "Returns md5sum for a message object of type '<nodeInfo-request>"
  "7752f5a8c2441ad5def7b480e5f4374d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'nodeInfo-request)))
  "Returns md5sum for a message object of type 'nodeInfo-request"
  "7752f5a8c2441ad5def7b480e5f4374d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<nodeInfo-request>)))
  "Returns full string definition for message of type '<nodeInfo-request>"
  (cl:format cl:nil "string nodeType # launch / node~%string packName # 包名~%string nodeName # .launch名字 / 节点名字~%string nodeState # start / stop~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'nodeInfo-request)))
  "Returns full string definition for message of type 'nodeInfo-request"
  (cl:format cl:nil "string nodeType # launch / node~%string packName # 包名~%string nodeName # .launch名字 / 节点名字~%string nodeState # start / stop~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <nodeInfo-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'nodeType))
     4 (cl:length (cl:slot-value msg 'packName))
     4 (cl:length (cl:slot-value msg 'nodeName))
     4 (cl:length (cl:slot-value msg 'nodeState))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <nodeInfo-request>))
  "Converts a ROS message object to a list"
  (cl:list 'nodeInfo-request
    (cl:cons ':nodeType (nodeType msg))
    (cl:cons ':packName (packName msg))
    (cl:cons ':nodeName (nodeName msg))
    (cl:cons ':nodeState (nodeState msg))
))
;//! \htmlinclude nodeInfo-response.msg.html

(cl:defclass <nodeInfo-response> (roslisp-msg-protocol:ros-message)
  ((ok
    :reader ok
    :initarg :ok
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass nodeInfo-response (<nodeInfo-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <nodeInfo-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'nodeInfo-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name node_manager-srv:<nodeInfo-response> is deprecated: use node_manager-srv:nodeInfo-response instead.")))

(cl:ensure-generic-function 'ok-val :lambda-list '(m))
(cl:defmethod ok-val ((m <nodeInfo-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader node_manager-srv:ok-val is deprecated.  Use node_manager-srv:ok instead.")
  (ok m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <nodeInfo-response>) ostream)
  "Serializes a message object of type '<nodeInfo-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ok) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <nodeInfo-response>) istream)
  "Deserializes a message object of type '<nodeInfo-response>"
    (cl:setf (cl:slot-value msg 'ok) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<nodeInfo-response>)))
  "Returns string type for a service object of type '<nodeInfo-response>"
  "node_manager/nodeInfoResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'nodeInfo-response)))
  "Returns string type for a service object of type 'nodeInfo-response"
  "node_manager/nodeInfoResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<nodeInfo-response>)))
  "Returns md5sum for a message object of type '<nodeInfo-response>"
  "7752f5a8c2441ad5def7b480e5f4374d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'nodeInfo-response)))
  "Returns md5sum for a message object of type 'nodeInfo-response"
  "7752f5a8c2441ad5def7b480e5f4374d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<nodeInfo-response>)))
  "Returns full string definition for message of type '<nodeInfo-response>"
  (cl:format cl:nil "bool ok~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'nodeInfo-response)))
  "Returns full string definition for message of type 'nodeInfo-response"
  (cl:format cl:nil "bool ok~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <nodeInfo-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <nodeInfo-response>))
  "Converts a ROS message object to a list"
  (cl:list 'nodeInfo-response
    (cl:cons ':ok (ok msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'nodeInfo)))
  'nodeInfo-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'nodeInfo)))
  'nodeInfo-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'nodeInfo)))
  "Returns string type for a service object of type '<nodeInfo>"
  "node_manager/nodeInfo")