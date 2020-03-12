;; Auto-generated. Do not edit!


(when (boundp 'data_handler::TestMessage)
  (if (not (find-package "DATA_HANDLER"))
    (make-package "DATA_HANDLER"))
  (shadow 'TestMessage (find-package "DATA_HANDLER")))
(unless (find-package "DATA_HANDLER::TESTMESSAGE")
  (make-package "DATA_HANDLER::TESTMESSAGE"))

(in-package "ROS")
;;//! \htmlinclude TestMessage.msg.html


(defclass data_handler::TestMessage
  :super ros::object
  :slots (_text _n _f ))

(defmethod data_handler::TestMessage
  (:init
   (&key
    ((:text __text) "")
    ((:n __n) 0)
    ((:f __f) 0.0)
    )
   (send-super :init)
   (setq _text (string __text))
   (setq _n (round __n))
   (setq _f (float __f))
   self)
  (:text
   (&optional __text)
   (if __text (setq _text __text)) _text)
  (:n
   (&optional __n)
   (if __n (setq _n __n)) _n)
  (:f
   (&optional __f)
   (if __f (setq _f __f)) _f)
  (:serialization-length
   ()
   (+
    ;; string _text
    4 (length _text)
    ;; int32 _n
    4
    ;; float32 _f
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _text
       (write-long (length _text) s) (princ _text s)
     ;; int32 _n
       (write-long _n s)
     ;; float32 _f
       (sys::poke _f (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _text
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _text (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int32 _n
     (setq _n (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _f
     (setq _f (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get data_handler::TestMessage :md5sum-) "abb3f28b1fb28ea97e948219b8896972")
(setf (get data_handler::TestMessage :datatype-) "data_handler/TestMessage")
(setf (get data_handler::TestMessage :definition-)
      "string text
int32 n
float32 f
")



(provide :data_handler/TestMessage "abb3f28b1fb28ea97e948219b8896972")


