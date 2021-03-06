;; Auto-generated. Do not edit!


(when (boundp 'amiro_msgs::Int32MultiArrayStamped)
  (if (not (find-package "AMIRO_MSGS"))
    (make-package "AMIRO_MSGS"))
  (shadow 'Int32MultiArrayStamped (find-package "AMIRO_MSGS")))
(unless (find-package "AMIRO_MSGS::INT32MULTIARRAYSTAMPED")
  (make-package "AMIRO_MSGS::INT32MULTIARRAYSTAMPED"))

(in-package "ROS")
;;//! \htmlinclude Int32MultiArrayStamped.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass amiro_msgs::Int32MultiArrayStamped
  :super ros::object
  :slots (_header _array ))

(defmethod amiro_msgs::Int32MultiArrayStamped
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:array __array) (instance std_msgs::Int32MultiArray :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _array __array)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:array
   (&rest __array)
   (if (keywordp (car __array))
       (send* _array __array)
     (progn
       (if __array (setq _array (car __array)))
       _array)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; std_msgs/Int32MultiArray _array
    (send _array :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; std_msgs/Int32MultiArray _array
       (send _array :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; std_msgs/Int32MultiArray _array
     (send _array :deserialize buf ptr-) (incf ptr- (send _array :serialization-length))
   ;;
   self)
  )

(setf (get amiro_msgs::Int32MultiArrayStamped :md5sum-) "a6518a46d079512f8a23203836b73fa9")
(setf (get amiro_msgs::Int32MultiArrayStamped :datatype-) "amiro_msgs/Int32MultiArrayStamped")
(setf (get amiro_msgs::Int32MultiArrayStamped :definition-)
      "# Stamped int32 multi array

std_msgs/Header header
std_msgs/Int32MultiArray array

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
MSG: std_msgs/Int32MultiArray
# Please look at the MultiArrayLayout message definition for
# documentation on all multiarrays.

MultiArrayLayout  layout        # specification of data layout
int32[]           data          # array of data


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



(provide :amiro_msgs/Int32MultiArrayStamped "a6518a46d079512f8a23203836b73fa9")


