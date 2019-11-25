
(cl:in-package :asdf)

(defsystem "lidar_camera_calibration-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "marker_6dof" :depends-on ("_package_marker_6dof"))
    (:file "_package_marker_6dof" :depends-on ("_package"))
  ))