;; Auto-generated. Do not edit!


(when (boundp 'yolo_example::DetectedObject)
  (if (not (find-package "YOLO_EXAMPLE"))
    (make-package "YOLO_EXAMPLE"))
  (shadow 'DetectedObject (find-package "YOLO_EXAMPLE")))
(unless (find-package "YOLO_EXAMPLE::DETECTEDOBJECT")
  (make-package "YOLO_EXAMPLE::DETECTEDOBJECT"))

(in-package "ROS")
;;//! \htmlinclude DetectedObject.msg.html


(defclass yolo_example::DetectedObject
  :super ros::object
  :slots (_label _confidence _box ))

(defmethod yolo_example::DetectedObject
  (:init
   (&key
    ((:label __label) "")
    ((:confidence __confidence) 0.0)
    ((:box __box) (instance yolo_example::BoundingBox :init))
    )
   (send-super :init)
   (setq _label (string __label))
   (setq _confidence (float __confidence))
   (setq _box __box)
   self)
  (:label
   (&optional __label)
   (if __label (setq _label __label)) _label)
  (:confidence
   (&optional __confidence)
   (if __confidence (setq _confidence __confidence)) _confidence)
  (:box
   (&rest __box)
   (if (keywordp (car __box))
       (send* _box __box)
     (progn
       (if __box (setq _box (car __box)))
       _box)))
  (:serialization-length
   ()
   (+
    ;; string _label
    4 (length _label)
    ;; float64 _confidence
    8
    ;; yolo_example/BoundingBox _box
    (send _box :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _label
       (write-long (length _label) s) (princ _label s)
     ;; float64 _confidence
       (sys::poke _confidence (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; yolo_example/BoundingBox _box
       (send _box :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _label
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _label (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float64 _confidence
     (setq _confidence (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; yolo_example/BoundingBox _box
     (send _box :deserialize buf ptr-) (incf ptr- (send _box :serialization-length))
   ;;
   self)
  )

(setf (get yolo_example::DetectedObject :md5sum-) "edf2e0356664286b429d6f21ae05e759")
(setf (get yolo_example::DetectedObject :datatype-) "yolo_example/DetectedObject")
(setf (get yolo_example::DetectedObject :definition-)
      "string label
float64 confidence
BoundingBox box
================================================================================
MSG: yolo_example/BoundingBox
float64 x1
float64 y1
float64 x2
float64 y2
")



(provide :yolo_example/DetectedObject "edf2e0356664286b429d6f21ae05e759")


