# header: ros header
# error_msg: user defined emergency stop message
# error code: We defined the rules. 0 is reserved for normal operation. for example,
# crash: 255,  perception failed:100-109, mapping failed: 110 - 119, planning failed: 120 - 129, etc,.

Header header
std_msgs/String error_msg
std_msgs/UInt8 error_code