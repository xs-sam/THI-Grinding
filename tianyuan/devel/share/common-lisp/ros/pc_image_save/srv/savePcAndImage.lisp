; Auto-generated. Do not edit!


(cl:in-package pc_image_save-srv)


;//! \htmlinclude savePcAndImage-request.msg.html

(cl:defclass <savePcAndImage-request> (roslisp-msg-protocol:ros-message)
  ((fileBaseName
    :reader fileBaseName
    :initarg :fileBaseName
    :type cl:string
    :initform "")
   (PointCloudFileType
    :reader PointCloudFileType
    :initarg :PointCloudFileType
    :type cl:string
    :initform ""))
)

(cl:defclass savePcAndImage-request (<savePcAndImage-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <savePcAndImage-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'savePcAndImage-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pc_image_save-srv:<savePcAndImage-request> is deprecated: use pc_image_save-srv:savePcAndImage-request instead.")))

(cl:ensure-generic-function 'fileBaseName-val :lambda-list '(m))
(cl:defmethod fileBaseName-val ((m <savePcAndImage-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pc_image_save-srv:fileBaseName-val is deprecated.  Use pc_image_save-srv:fileBaseName instead.")
  (fileBaseName m))

(cl:ensure-generic-function 'PointCloudFileType-val :lambda-list '(m))
(cl:defmethod PointCloudFileType-val ((m <savePcAndImage-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pc_image_save-srv:PointCloudFileType-val is deprecated.  Use pc_image_save-srv:PointCloudFileType instead.")
  (PointCloudFileType m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <savePcAndImage-request>) ostream)
  "Serializes a message object of type '<savePcAndImage-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'fileBaseName))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'fileBaseName))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'PointCloudFileType))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'PointCloudFileType))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <savePcAndImage-request>) istream)
  "Deserializes a message object of type '<savePcAndImage-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'fileBaseName) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'fileBaseName) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'PointCloudFileType) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'PointCloudFileType) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<savePcAndImage-request>)))
  "Returns string type for a service object of type '<savePcAndImage-request>"
  "pc_image_save/savePcAndImageRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'savePcAndImage-request)))
  "Returns string type for a service object of type 'savePcAndImage-request"
  "pc_image_save/savePcAndImageRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<savePcAndImage-request>)))
  "Returns md5sum for a message object of type '<savePcAndImage-request>"
  "dd3b3b413ecc248d44e5417f1e4d808b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'savePcAndImage-request)))
  "Returns md5sum for a message object of type 'savePcAndImage-request"
  "dd3b3b413ecc248d44e5417f1e4d808b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<savePcAndImage-request>)))
  "Returns full string definition for message of type '<savePcAndImage-request>"
  (cl:format cl:nil "string fileBaseName~%string PointCloudFileType~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'savePcAndImage-request)))
  "Returns full string definition for message of type 'savePcAndImage-request"
  (cl:format cl:nil "string fileBaseName~%string PointCloudFileType~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <savePcAndImage-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'fileBaseName))
     4 (cl:length (cl:slot-value msg 'PointCloudFileType))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <savePcAndImage-request>))
  "Converts a ROS message object to a list"
  (cl:list 'savePcAndImage-request
    (cl:cons ':fileBaseName (fileBaseName msg))
    (cl:cons ':PointCloudFileType (PointCloudFileType msg))
))
;//! \htmlinclude savePcAndImage-response.msg.html

(cl:defclass <savePcAndImage-response> (roslisp-msg-protocol:ros-message)
  ((ok
    :reader ok
    :initarg :ok
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass savePcAndImage-response (<savePcAndImage-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <savePcAndImage-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'savePcAndImage-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pc_image_save-srv:<savePcAndImage-response> is deprecated: use pc_image_save-srv:savePcAndImage-response instead.")))

(cl:ensure-generic-function 'ok-val :lambda-list '(m))
(cl:defmethod ok-val ((m <savePcAndImage-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pc_image_save-srv:ok-val is deprecated.  Use pc_image_save-srv:ok instead.")
  (ok m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <savePcAndImage-response>) ostream)
  "Serializes a message object of type '<savePcAndImage-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ok) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <savePcAndImage-response>) istream)
  "Deserializes a message object of type '<savePcAndImage-response>"
    (cl:setf (cl:slot-value msg 'ok) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<savePcAndImage-response>)))
  "Returns string type for a service object of type '<savePcAndImage-response>"
  "pc_image_save/savePcAndImageResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'savePcAndImage-response)))
  "Returns string type for a service object of type 'savePcAndImage-response"
  "pc_image_save/savePcAndImageResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<savePcAndImage-response>)))
  "Returns md5sum for a message object of type '<savePcAndImage-response>"
  "dd3b3b413ecc248d44e5417f1e4d808b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'savePcAndImage-response)))
  "Returns md5sum for a message object of type 'savePcAndImage-response"
  "dd3b3b413ecc248d44e5417f1e4d808b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<savePcAndImage-response>)))
  "Returns full string definition for message of type '<savePcAndImage-response>"
  (cl:format cl:nil "bool ok~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'savePcAndImage-response)))
  "Returns full string definition for message of type 'savePcAndImage-response"
  (cl:format cl:nil "bool ok~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <savePcAndImage-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <savePcAndImage-response>))
  "Converts a ROS message object to a list"
  (cl:list 'savePcAndImage-response
    (cl:cons ':ok (ok msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'savePcAndImage)))
  'savePcAndImage-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'savePcAndImage)))
  'savePcAndImage-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'savePcAndImage)))
  "Returns string type for a service object of type '<savePcAndImage>"
  "pc_image_save/savePcAndImage")