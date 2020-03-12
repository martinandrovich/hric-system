; Auto-generated. Do not edit!


(cl:in-package data_handler-msg)


;//! \htmlinclude TestMessage.msg.html

(cl:defclass <TestMessage> (roslisp-msg-protocol:ros-message)
  ((text
    :reader text
    :initarg :text
    :type cl:string
    :initform "")
   (n
    :reader n
    :initarg :n
    :type cl:integer
    :initform 0)
   (f
    :reader f
    :initarg :f
    :type cl:float
    :initform 0.0))
)

(cl:defclass TestMessage (<TestMessage>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TestMessage>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TestMessage)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name data_handler-msg:<TestMessage> is deprecated: use data_handler-msg:TestMessage instead.")))

(cl:ensure-generic-function 'text-val :lambda-list '(m))
(cl:defmethod text-val ((m <TestMessage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader data_handler-msg:text-val is deprecated.  Use data_handler-msg:text instead.")
  (text m))

(cl:ensure-generic-function 'n-val :lambda-list '(m))
(cl:defmethod n-val ((m <TestMessage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader data_handler-msg:n-val is deprecated.  Use data_handler-msg:n instead.")
  (n m))

(cl:ensure-generic-function 'f-val :lambda-list '(m))
(cl:defmethod f-val ((m <TestMessage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader data_handler-msg:f-val is deprecated.  Use data_handler-msg:f instead.")
  (f m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TestMessage>) ostream)
  "Serializes a message object of type '<TestMessage>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'text))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'text))
  (cl:let* ((signed (cl:slot-value msg 'n)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'f))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TestMessage>) istream)
  "Deserializes a message object of type '<TestMessage>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'text) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'text) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'n) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'f) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TestMessage>)))
  "Returns string type for a message object of type '<TestMessage>"
  "data_handler/TestMessage")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TestMessage)))
  "Returns string type for a message object of type 'TestMessage"
  "data_handler/TestMessage")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TestMessage>)))
  "Returns md5sum for a message object of type '<TestMessage>"
  "abb3f28b1fb28ea97e948219b8896972")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TestMessage)))
  "Returns md5sum for a message object of type 'TestMessage"
  "abb3f28b1fb28ea97e948219b8896972")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TestMessage>)))
  "Returns full string definition for message of type '<TestMessage>"
  (cl:format cl:nil "string text~%int32 n~%float32 f~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TestMessage)))
  "Returns full string definition for message of type 'TestMessage"
  (cl:format cl:nil "string text~%int32 n~%float32 f~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TestMessage>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'text))
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TestMessage>))
  "Converts a ROS message object to a list"
  (cl:list 'TestMessage
    (cl:cons ':text (text msg))
    (cl:cons ':n (n msg))
    (cl:cons ':f (f msg))
))
