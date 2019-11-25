;; Auto-generated. Do not edit!


(when (boundp 'lidar_camera_calibration::marker_6dof)
  (if (not (find-package "LIDAR_CAMERA_CALIBRATION"))
    (make-package "LIDAR_CAMERA_CALIBRATION"))
  (shadow 'marker_6dof (find-package "LIDAR_CAMERA_CALIBRATION")))
(unless (find-package "LIDAR_CAMERA_CALIBRATION::MARKER_6DOF")
  (make-package "LIDAR_CAMERA_CALIBRATION::MARKER_6DOF"))

(in-package "ROS")
;;//! \htmlinclude marker_6dof.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass lidar_camera_calibration::marker_6dof
  :super ros::object
  :slots (_header _num_of_markers _dof ))

(defmethod lidar_camera_calibration::marker_6dof
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:num_of_markers __num_of_markers) 0)
    ((:dof __dof) (instance std_msgs::Float32MultiArray :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _num_of_markers (round __num_of_markers))
   (setq _dof __dof)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:num_of_markers
   (&optional __num_of_markers)
   (if __num_of_markers (setq _num_of_markers __num_of_markers)) _num_of_markers)
  (:dof
   (&rest __dof)
   (if (keywordp (car __dof))
       (send* _dof __dof)
     (progn
       (if __dof (setq _dof (car __dof)))
       _dof)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; int32 _num_of_markers
    4
    ;; std_msgs/Float32MultiArray _dof
    (send _dof :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; int32 _num_of_markers
       (write-long _num_of_markers s)
     ;; std_msgs/Float32MultiArray _dof
       (send _dof :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; int32 _num_of_markers
     (setq _num_of_markers (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; std_msgs/Float32MultiArray _dof
     (send _dof :deserialize buf ptr-) (incf ptr- (send _dof :serialization-length))
   ;;
   self)
  )

(setf (get lidar_camera_calibration::marker_6dof :md5sum-) "b58090eb705a228fefaefd143c65c540")
(setf (get lidar_camera_calibration::marker_6dof :datatype-) "lidar_camera_calibration/marker_6dof")
(setf (get lidar_camera_calibration::marker_6dof :definition-)
      "std_msgs/Header header
int32 num_of_markers
std_msgs/Float32MultiArray dof
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: std_msgs/Float32MultiArray
# Please look at the MultiArrayLayout message definition for
# documentation on all multiarrays.

MultiArrayLayout  layout        # specification of data layout
float32[]         data          # array of data


================================================================================
MSG: std_msgs/MultiArrayLayout
# The multiarray declares a generic multi-dimensional array of a
# particular data type.  Dimensions are ordered from outer most
# to inner most.

MultiArrayDimension[] dim # Array of dimension properties
uint32 data_offset        # padding elements at front of data

# Accessors should ALWAYS be written in terms of dimension stride
# and specified outer-most dimension first.
# 
# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]
#
# A standard, 3-channel 640x480 image with interleaved color channels
# would be specified as:
#
# dim[0].label  = \"height\"
# dim[0].size   = 480
# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)
# dim[1].label  = \"width\"
# dim[1].size   = 640
# dim[1].stride = 3*640 = 1920
# dim[2].label  = \"channel\"
# dim[2].size   = 3
# dim[2].stride = 3
#
# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.

================================================================================
MSG: std_msgs/MultiArrayDimension
string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
uint32 stride  # stride of given dimension
")



(provide :lidar_camera_calibration/marker_6dof "b58090eb705a228fefaefd143c65c540")


