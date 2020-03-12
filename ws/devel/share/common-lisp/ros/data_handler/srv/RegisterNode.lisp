; Auto-generated. Do not edit!


(cl:in-package data_handler-srv)


;//! \htmlinclude RegisterNode-request.msg.html

(cl:defclass <RegisterNode-request> (roslisp-msg-protocol:ros-message)
  ((name
    :reader name
    :initarg :name
    :type cl:string
    :initform "")
   (topic
    :reader topic
    :initarg :topic
    :type cl:string
    :initform "")
   (hz
    :reader hz
    :initarg :hz
    :type cl:integer
    :initform 0)
   (use_dynamic_freq
    :reader use_dynamic_freq
    :initarg :use_dynamic_freq
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass RegisterNode-request (<RegisterNode-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RegisterNode-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RegisterNode-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name data_handler-srv:<RegisterNode-request> is deprecated: use data_handler-srv:RegisterNode-request instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <RegisterNode-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader data_handler-srv:name-val is deprecated.  Use data_handler-srv:name instead.")
  (name m))

(cl:ensure-generic-function 'topic-val :lambda-list '(m))
(cl:defmethod topic-val ((m <RegisterNode-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader data_handler-srv:topic-val is deprecated.  Use data_handler-srv:topic instead.")
  (topic m))

(cl:ensure-generic-function 'hz-val :lambda-list '(m))
(cl:defmethod hz-val ((m <RegisterNode-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader data_handler-srv:hz-val is deprecated.  Use data_handler-srv:hz instead.")
  (hz m))

(cl:ensure-generic-function 'use_dynamic_freq-val :lambda-list '(m))
(cl:defmethod use_dynamic_freq-val ((m <RegisterNode-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader data_handler-srv:use_dynamic_freq-val is deprecated.  Use data_handler-srv:use_dynamic_freq instead.")
  (use_dynamic_freq m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RegisterNode-request>) ostream)
  "Serializes a message object of type '<RegisterNode-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'topic))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'topic))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'hz)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'hz)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'hz)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'hz)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 32) (cl:slot-value msg 'hz)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 40) (cl:slot-value msg 'hz)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 48) (cl:slot-value msg 'hz)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 56) (cl:slot-value msg 'hz)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'use_dynamic_freq) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RegisterNode-request>) istream)
  "Deserializes a message object of type '<RegisterNode-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'topic) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'topic) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'hz)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'hz)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'hz)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'hz)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 32) (cl:slot-value msg 'hz)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 40) (cl:slot-value msg 'hz)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 48) (cl:slot-value msg 'hz)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 56) (cl:slot-value msg 'hz)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'use_dynamic_freq) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RegisterNode-request>)))
  "Returns string type for a service object of type '<RegisterNode-request>"
  "data_handler/RegisterNodeRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RegisterNode-request)))
  "Returns string type for a service object of type 'RegisterNode-request"
  "data_handler/RegisterNodeRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RegisterNode-request>)))
  "Returns md5sum for a message object of type '<RegisterNode-request>"
  "0df1a079a9a4e3ccb2fd3852ded39c15")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RegisterNode-request)))
  "Returns md5sum for a message object of type 'RegisterNode-request"
  "0df1a079a9a4e3ccb2fd3852ded39c15")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RegisterNode-request>)))
  "Returns full string definition for message of type '<RegisterNode-request>"
  (cl:format cl:nil "string name~%string topic~%uint64 hz~%bool use_dynamic_freq~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RegisterNode-request)))
  "Returns full string definition for message of type 'RegisterNode-request"
  (cl:format cl:nil "string name~%string topic~%uint64 hz~%bool use_dynamic_freq~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RegisterNode-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'name))
     4 (cl:length (cl:slot-value msg 'topic))
     8
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RegisterNode-request>))
  "Converts a ROS message object to a list"
  (cl:list 'RegisterNode-request
    (cl:cons ':name (name msg))
    (cl:cons ':topic (topic msg))
    (cl:cons ':hz (hz msg))
    (cl:cons ':use_dynamic_freq (use_dynamic_freq msg))
))
;//! \htmlinclude RegisterNode-response.msg.html

(cl:defclass <RegisterNode-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass RegisterNode-response (<RegisterNode-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RegisterNode-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RegisterNode-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name data_handler-srv:<RegisterNode-response> is deprecated: use data_handler-srv:RegisterNode-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <RegisterNode-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader data_handler-srv:success-val is deprecated.  Use data_handler-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RegisterNode-response>) ostream)
  "Serializes a message object of type '<RegisterNode-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RegisterNode-response>) istream)
  "Deserializes a message object of type '<RegisterNode-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RegisterNode-response>)))
  "Returns string type for a service object of type '<RegisterNode-response>"
  "data_handler/RegisterNodeResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RegisterNode-response)))
  "Returns string type for a service object of type 'RegisterNode-response"
  "data_handler/RegisterNodeResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RegisterNode-response>)))
  "Returns md5sum for a message object of type '<RegisterNode-response>"
  "0df1a079a9a4e3ccb2fd3852ded39c15")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RegisterNode-response)))
  "Returns md5sum for a message object of type 'RegisterNode-response"
  "0df1a079a9a4e3ccb2fd3852ded39c15")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RegisterNode-response>)))
  "Returns full string definition for message of type '<RegisterNode-response>"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RegisterNode-response)))
  "Returns full string definition for message of type 'RegisterNode-response"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RegisterNode-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RegisterNode-response>))
  "Converts a ROS message object to a list"
  (cl:list 'RegisterNode-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'RegisterNode)))
  'RegisterNode-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'RegisterNode)))
  'RegisterNode-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RegisterNode)))
  "Returns string type for a service object of type '<RegisterNode>"
  "data_handler/RegisterNode")