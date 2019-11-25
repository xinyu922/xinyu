; Auto-generated. Do not edit!


(cl:in-package lidar_camera_calibration-msg)


;//! \htmlinclude marker_6dof.msg.html

(cl:defclass <marker_6dof> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (num_of_markers
    :reader num_of_markers
    :initarg :num_of_markers
    :type cl:integer
    :initform 0)
   (dof
    :reader dof
    :initarg :dof
    :type std_msgs-msg:Float32MultiArray
    :initform (cl:make-instance 'std_msgs-msg:Float32MultiArray)))
)

(cl:defclass marker_6dof (<marker_6dof>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <marker_6dof>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'marker_6dof)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name lidar_camera_calibration-msg:<marker_6dof> is deprecated: use lidar_camera_calibration-msg:marker_6dof instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <marker_6dof>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lidar_camera_calibration-msg:header-val is deprecated.  Use lidar_camera_calibration-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'num_of_markers-val :lambda-list '(m))
(cl:defmethod num_of_markers-val ((m <marker_6dof>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lidar_camera_calibration-msg:num_of_markers-val is deprecated.  Use lidar_camera_calibration-msg:num_of_markers instead.")
  (num_of_markers m))

(cl:ensure-generic-function 'dof-val :lambda-list '(m))
(cl:defmethod dof-val ((m <marker_6dof>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lidar_camera_calibration-msg:dof-val is deprecated.  Use lidar_camera_calibration-msg:dof instead.")
  (dof m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <marker_6dof>) ostream)
  "Serializes a message object of type '<marker_6dof>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let* ((signed (cl:slot-value msg 'num_of_markers)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'dof) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <marker_6dof>) istream)
  "Deserializes a message object of type '<marker_6dof>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'num_of_markers) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'dof) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<marker_6dof>)))
  "Returns string type for a message object of type '<marker_6dof>"
  "lidar_camera_calibration/marker_6dof")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'marker_6dof)))
  "Returns string type for a message object of type 'marker_6dof"
  "lidar_camera_calibration/marker_6dof")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<marker_6dof>)))
  "Returns md5sum for a message object of type '<marker_6dof>"
  "b58090eb705a228fefaefd143c65c540")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'marker_6dof)))
  "Returns md5sum for a message object of type 'marker_6dof"
  "b58090eb705a228fefaefd143c65c540")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<marker_6dof>)))
  "Returns full string definition for message of type '<marker_6dof>"
  (cl:format cl:nil "std_msgs/Header header~%int32 num_of_markers~%std_msgs/Float32MultiArray dof~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: std_msgs/Float32MultiArray~%# Please look at the MultiArrayLayout message definition for~%# documentation on all multiarrays.~%~%MultiArrayLayout  layout        # specification of data layout~%float32[]         data          # array of data~%~%~%================================================================================~%MSG: std_msgs/MultiArrayLayout~%# The multiarray declares a generic multi-dimensional array of a~%# particular data type.  Dimensions are ordered from outer most~%# to inner most.~%~%MultiArrayDimension[] dim # Array of dimension properties~%uint32 data_offset        # padding elements at front of data~%~%# Accessors should ALWAYS be written in terms of dimension stride~%# and specified outer-most dimension first.~%# ~%# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]~%#~%# A standard, 3-channel 640x480 image with interleaved color channels~%# would be specified as:~%#~%# dim[0].label  = \"height\"~%# dim[0].size   = 480~%# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)~%# dim[1].label  = \"width\"~%# dim[1].size   = 640~%# dim[1].stride = 3*640 = 1920~%# dim[2].label  = \"channel\"~%# dim[2].size   = 3~%# dim[2].stride = 3~%#~%# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.~%~%================================================================================~%MSG: std_msgs/MultiArrayDimension~%string label   # label of given dimension~%uint32 size    # size of given dimension (in type units)~%uint32 stride  # stride of given dimension~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'marker_6dof)))
  "Returns full string definition for message of type 'marker_6dof"
  (cl:format cl:nil "std_msgs/Header header~%int32 num_of_markers~%std_msgs/Float32MultiArray dof~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: std_msgs/Float32MultiArray~%# Please look at the MultiArrayLayout message definition for~%# documentation on all multiarrays.~%~%MultiArrayLayout  layout        # specification of data layout~%float32[]         data          # array of data~%~%~%================================================================================~%MSG: std_msgs/MultiArrayLayout~%# The multiarray declares a generic multi-dimensional array of a~%# particular data type.  Dimensions are ordered from outer most~%# to inner most.~%~%MultiArrayDimension[] dim # Array of dimension properties~%uint32 data_offset        # padding elements at front of data~%~%# Accessors should ALWAYS be written in terms of dimension stride~%# and specified outer-most dimension first.~%# ~%# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]~%#~%# A standard, 3-channel 640x480 image with interleaved color channels~%# would be specified as:~%#~%# dim[0].label  = \"height\"~%# dim[0].size   = 480~%# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)~%# dim[1].label  = \"width\"~%# dim[1].size   = 640~%# dim[1].stride = 3*640 = 1920~%# dim[2].label  = \"channel\"~%# dim[2].size   = 3~%# dim[2].stride = 3~%#~%# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.~%~%================================================================================~%MSG: std_msgs/MultiArrayDimension~%string label   # label of given dimension~%uint32 size    # size of given dimension (in type units)~%uint32 stride  # stride of given dimension~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <marker_6dof>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'dof))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <marker_6dof>))
  "Converts a ROS message object to a list"
  (cl:list 'marker_6dof
    (cl:cons ':header (header msg))
    (cl:cons ':num_of_markers (num_of_markers msg))
    (cl:cons ':dof (dof msg))
))
