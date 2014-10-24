; Auto-generated. Do not edit!


(cl:in-package rsd_camera_functions-msg)


;//! \htmlinclude line_points.msg.html

(cl:defclass <line_points> (roslisp-msg-protocol:ros-message)
  ((x1
    :reader x1
    :initarg :x1
    :type cl:float
    :initform 0.0)
   (y1
    :reader y1
    :initarg :y1
    :type cl:float
    :initform 0.0)
   (x2
    :reader x2
    :initarg :x2
    :type cl:float
    :initform 0.0)
   (y2
    :reader y2
    :initarg :y2
    :type cl:float
    :initform 0.0))
)

(cl:defclass line_points (<line_points>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <line_points>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'line_points)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rsd_camera_functions-msg:<line_points> is deprecated: use rsd_camera_functions-msg:line_points instead.")))

(cl:ensure-generic-function 'x1-val :lambda-list '(m))
(cl:defmethod x1-val ((m <line_points>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rsd_camera_functions-msg:x1-val is deprecated.  Use rsd_camera_functions-msg:x1 instead.")
  (x1 m))

(cl:ensure-generic-function 'y1-val :lambda-list '(m))
(cl:defmethod y1-val ((m <line_points>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rsd_camera_functions-msg:y1-val is deprecated.  Use rsd_camera_functions-msg:y1 instead.")
  (y1 m))

(cl:ensure-generic-function 'x2-val :lambda-list '(m))
(cl:defmethod x2-val ((m <line_points>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rsd_camera_functions-msg:x2-val is deprecated.  Use rsd_camera_functions-msg:x2 instead.")
  (x2 m))

(cl:ensure-generic-function 'y2-val :lambda-list '(m))
(cl:defmethod y2-val ((m <line_points>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rsd_camera_functions-msg:y2-val is deprecated.  Use rsd_camera_functions-msg:y2 instead.")
  (y2 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <line_points>) ostream)
  "Serializes a message object of type '<line_points>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'y1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'y2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <line_points>) istream)
  "Deserializes a message object of type '<line_points>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x1) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y1) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x2) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y2) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<line_points>)))
  "Returns string type for a message object of type '<line_points>"
  "rsd_camera_functions/line_points")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'line_points)))
  "Returns string type for a message object of type 'line_points"
  "rsd_camera_functions/line_points")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<line_points>)))
  "Returns md5sum for a message object of type '<line_points>"
  "1d74979d8119401281d48677f845994f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'line_points)))
  "Returns md5sum for a message object of type 'line_points"
  "1d74979d8119401281d48677f845994f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<line_points>)))
  "Returns full string definition for message of type '<line_points>"
  (cl:format cl:nil "float32 x1~%float32 y1~%float32 x2~%float32 y2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'line_points)))
  "Returns full string definition for message of type 'line_points"
  (cl:format cl:nil "float32 x1~%float32 y1~%float32 x2~%float32 y2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <line_points>))
  (cl:+ 0
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <line_points>))
  "Converts a ROS message object to a list"
  (cl:list 'line_points
    (cl:cons ':x1 (x1 msg))
    (cl:cons ':y1 (y1 msg))
    (cl:cons ':x2 (x2 msg))
    (cl:cons ':y2 (y2 msg))
))
