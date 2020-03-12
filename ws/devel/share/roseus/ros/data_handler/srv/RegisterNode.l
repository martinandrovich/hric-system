;; Auto-generated. Do not edit!


(when (boundp 'data_handler::RegisterNode)
  (if (not (find-package "DATA_HANDLER"))
    (make-package "DATA_HANDLER"))
  (shadow 'RegisterNode (find-package "DATA_HANDLER")))
(unless (find-package "DATA_HANDLER::REGISTERNODE")
  (make-package "DATA_HANDLER::REGISTERNODE"))
(unless (find-package "DATA_HANDLER::REGISTERNODEREQUEST")
  (make-package "DATA_HANDLER::REGISTERNODEREQUEST"))
(unless (find-package "DATA_HANDLER::REGISTERNODERESPONSE")
  (make-package "DATA_HANDLER::REGISTERNODERESPONSE"))

(in-package "ROS")





(defclass data_handler::RegisterNodeRequest
  :super ros::object
  :slots (_name _topic _hz _use_dynamic_freq ))

(defmethod data_handler::RegisterNodeRequest
  (:init
   (&key
    ((:name __name) "")
    ((:topic __topic) "")
    ((:hz __hz) 0)
    ((:use_dynamic_freq __use_dynamic_freq) nil)
    )
   (send-super :init)
   (setq _name (string __name))
   (setq _topic (string __topic))
   (setq _hz (round __hz))
   (setq _use_dynamic_freq __use_dynamic_freq)
   self)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:topic
   (&optional __topic)
   (if __topic (setq _topic __topic)) _topic)
  (:hz
   (&optional __hz)
   (if __hz (setq _hz __hz)) _hz)
  (:use_dynamic_freq
   (&optional __use_dynamic_freq)
   (if __use_dynamic_freq (setq _use_dynamic_freq __use_dynamic_freq)) _use_dynamic_freq)
  (:serialization-length
   ()
   (+
    ;; string _name
    4 (length _name)
    ;; string _topic
    4 (length _topic)
    ;; uint64 _hz
    8
    ;; bool _use_dynamic_freq
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;; string _topic
       (write-long (length _topic) s) (princ _topic s)
     ;; uint64 _hz
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _hz (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _hz) (= (length (_hz . bv)) 2)) ;; bignum
              (write-long (ash (elt (_hz . bv) 0) 0) s)
              (write-long (ash (elt (_hz . bv) 1) -1) s))
             ((and (class _hz) (= (length (_hz . bv)) 1)) ;; big1
              (write-long (elt (_hz . bv) 0) s)
              (write-long (if (>= _hz 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _hz s)(write-long (if (>= _hz 0) 0 #xffffffff) s)))
     ;; bool _use_dynamic_freq
       (if _use_dynamic_freq (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _topic
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _topic (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; uint64 _hz
     
#+(or :alpha :irix6 :x86_64)
      (setf _hz (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _hz (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; bool _use_dynamic_freq
     (setq _use_dynamic_freq (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass data_handler::RegisterNodeResponse
  :super ros::object
  :slots (_success ))

(defmethod data_handler::RegisterNodeResponse
  (:init
   (&key
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _success __success)
   self)
  (:success
   (&optional __success)
   (if __success (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass data_handler::RegisterNode
  :super ros::object
  :slots ())

(setf (get data_handler::RegisterNode :md5sum-) "0df1a079a9a4e3ccb2fd3852ded39c15")
(setf (get data_handler::RegisterNode :datatype-) "data_handler/RegisterNode")
(setf (get data_handler::RegisterNode :request) data_handler::RegisterNodeRequest)
(setf (get data_handler::RegisterNode :response) data_handler::RegisterNodeResponse)

(defmethod data_handler::RegisterNodeRequest
  (:response () (instance data_handler::RegisterNodeResponse :init)))

(setf (get data_handler::RegisterNodeRequest :md5sum-) "0df1a079a9a4e3ccb2fd3852ded39c15")
(setf (get data_handler::RegisterNodeRequest :datatype-) "data_handler/RegisterNodeRequest")
(setf (get data_handler::RegisterNodeRequest :definition-)
      "string name
string topic
uint64 hz
bool use_dynamic_freq
---
bool success
")

(setf (get data_handler::RegisterNodeResponse :md5sum-) "0df1a079a9a4e3ccb2fd3852ded39c15")
(setf (get data_handler::RegisterNodeResponse :datatype-) "data_handler/RegisterNodeResponse")
(setf (get data_handler::RegisterNodeResponse :definition-)
      "string name
string topic
uint64 hz
bool use_dynamic_freq
---
bool success
")



(provide :data_handler/RegisterNode "0df1a079a9a4e3ccb2fd3852ded39c15")


