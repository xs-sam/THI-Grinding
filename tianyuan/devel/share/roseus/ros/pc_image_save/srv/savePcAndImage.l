;; Auto-generated. Do not edit!


(when (boundp 'pc_image_save::savePcAndImage)
  (if (not (find-package "PC_IMAGE_SAVE"))
    (make-package "PC_IMAGE_SAVE"))
  (shadow 'savePcAndImage (find-package "PC_IMAGE_SAVE")))
(unless (find-package "PC_IMAGE_SAVE::SAVEPCANDIMAGE")
  (make-package "PC_IMAGE_SAVE::SAVEPCANDIMAGE"))
(unless (find-package "PC_IMAGE_SAVE::SAVEPCANDIMAGEREQUEST")
  (make-package "PC_IMAGE_SAVE::SAVEPCANDIMAGEREQUEST"))
(unless (find-package "PC_IMAGE_SAVE::SAVEPCANDIMAGERESPONSE")
  (make-package "PC_IMAGE_SAVE::SAVEPCANDIMAGERESPONSE"))

(in-package "ROS")





(defclass pc_image_save::savePcAndImageRequest
  :super ros::object
  :slots (_fileBaseName _PointCloudFileType ))

(defmethod pc_image_save::savePcAndImageRequest
  (:init
   (&key
    ((:fileBaseName __fileBaseName) "")
    ((:PointCloudFileType __PointCloudFileType) "")
    )
   (send-super :init)
   (setq _fileBaseName (string __fileBaseName))
   (setq _PointCloudFileType (string __PointCloudFileType))
   self)
  (:fileBaseName
   (&optional __fileBaseName)
   (if __fileBaseName (setq _fileBaseName __fileBaseName)) _fileBaseName)
  (:PointCloudFileType
   (&optional __PointCloudFileType)
   (if __PointCloudFileType (setq _PointCloudFileType __PointCloudFileType)) _PointCloudFileType)
  (:serialization-length
   ()
   (+
    ;; string _fileBaseName
    4 (length _fileBaseName)
    ;; string _PointCloudFileType
    4 (length _PointCloudFileType)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _fileBaseName
       (write-long (length _fileBaseName) s) (princ _fileBaseName s)
     ;; string _PointCloudFileType
       (write-long (length _PointCloudFileType) s) (princ _PointCloudFileType s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _fileBaseName
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _fileBaseName (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _PointCloudFileType
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _PointCloudFileType (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass pc_image_save::savePcAndImageResponse
  :super ros::object
  :slots (_ok ))

(defmethod pc_image_save::savePcAndImageResponse
  (:init
   (&key
    ((:ok __ok) nil)
    )
   (send-super :init)
   (setq _ok __ok)
   self)
  (:ok
   (&optional (__ok :null))
   (if (not (eq __ok :null)) (setq _ok __ok)) _ok)
  (:serialization-length
   ()
   (+
    ;; bool _ok
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _ok
       (if _ok (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _ok
     (setq _ok (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass pc_image_save::savePcAndImage
  :super ros::object
  :slots ())

(setf (get pc_image_save::savePcAndImage :md5sum-) "dd3b3b413ecc248d44e5417f1e4d808b")
(setf (get pc_image_save::savePcAndImage :datatype-) "pc_image_save/savePcAndImage")
(setf (get pc_image_save::savePcAndImage :request) pc_image_save::savePcAndImageRequest)
(setf (get pc_image_save::savePcAndImage :response) pc_image_save::savePcAndImageResponse)

(defmethod pc_image_save::savePcAndImageRequest
  (:response () (instance pc_image_save::savePcAndImageResponse :init)))

(setf (get pc_image_save::savePcAndImageRequest :md5sum-) "dd3b3b413ecc248d44e5417f1e4d808b")
(setf (get pc_image_save::savePcAndImageRequest :datatype-) "pc_image_save/savePcAndImageRequest")
(setf (get pc_image_save::savePcAndImageRequest :definition-)
      "string fileBaseName
string PointCloudFileType
---
bool ok

")

(setf (get pc_image_save::savePcAndImageResponse :md5sum-) "dd3b3b413ecc248d44e5417f1e4d808b")
(setf (get pc_image_save::savePcAndImageResponse :datatype-) "pc_image_save/savePcAndImageResponse")
(setf (get pc_image_save::savePcAndImageResponse :definition-)
      "string fileBaseName
string PointCloudFileType
---
bool ok

")



(provide :pc_image_save/savePcAndImage "dd3b3b413ecc248d44e5417f1e4d808b")


