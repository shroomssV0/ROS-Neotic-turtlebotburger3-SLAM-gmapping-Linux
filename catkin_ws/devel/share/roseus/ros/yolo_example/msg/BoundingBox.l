;; Auto-generated. Do not edit!


(when (boundp 'yolo_example::BoundingBox)
  (if (not (find-package "YOLO_EXAMPLE"))
    (make-package "YOLO_EXAMPLE"))
  (shadow 'BoundingBox (find-package "YOLO_EXAMPLE")))
(unless (find-package "YOLO_EXAMPLE::BOUNDINGBOX")
  (make-package "YOLO_EXAMPLE::BOUNDINGBOX"))

(in-package "ROS")
;;//! \htmlinclude BoundingBox.msg.html


(defclass yolo_example::BoundingBox
  :super ros::object
  :slots (_x1 _y1 _x2 _y2 ))

(defmethod yolo_example::BoundingBox
  (:init
   (&key
    ((:x1 __x1) 0.0)
    ((:y1 __y1) 0.0)
    ((:x2 __x2) 0.0)
    ((:y2 __y2) 0.0)
    )
   (send-super :init)
   (setq _x1 (float __x1))
   (setq _y1 (float __y1))
   (setq _x2 (float __x2))
   (setq _y2 (float __y2))
   self)
  (:x1
   (&optional __x1)
   (if __x1 (setq _x1 __x1)) _x1)
  (:y1
   (&optional __y1)
   (if __y1 (setq _y1 __y1)) _y1)
  (:x2
   (&optional __x2)
   (if __x2 (setq _x2 __x2)) _x2)
  (:y2
   (&optional __y2)
   (if __y2 (setq _y2 __y2)) _y2)
  (:serialization-length
   ()
   (+
    ;; float64 _x1
    8
    ;; float64 _y1
    8
    ;; float64 _x2
    8
    ;; float64 _y2
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _x1
       (sys::poke _x1 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _y1
       (sys::poke _y1 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _x2
       (sys::poke _x2 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _y2
       (sys::poke _y2 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _x1
     (setq _x1 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y1
     (setq _y1 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _x2
     (setq _x2 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y2
     (setq _y2 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get yolo_example::BoundingBox :md5sum-) "3010fad09b09b8d3fdecdd94d144f7a1")
(setf (get yolo_example::BoundingBox :datatype-) "yolo_example/BoundingBox")
(setf (get yolo_example::BoundingBox :definition-)
      "float64 x1
float64 y1
float64 x2
float64 y2
")



(provide :yolo_example/BoundingBox "3010fad09b09b8d3fdecdd94d144f7a1")


