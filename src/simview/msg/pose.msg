int8 STATUS_INVALID=-1
int8 STATUS_UNSTABLE=1
int8 STATUS_VALID=2

std_msgs/Header header
float64 utc_time
float64 x
float64 y
float64 heading     # degree, zero to east, anticlockwise
float64 pitch 
float64 roll
float64 speed       # m/s
float64 heading_v   # degree/s, same to heading
float64[2] position_stddev
float64 heading_stddev
int8 sate_num
int8 status 