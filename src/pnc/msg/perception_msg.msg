#sick lidar data
float32[] sick_x
float32[] sick_y

#前16线转化为与单线相同消息
float32[] lidar16_front_x
float32[] lidar16_front_y

#后16线
uint8 rear_lidar

#视觉和毫米波
float32[] vision_x
float32[] vision_y

#ultrasound
uint8[] ultrasoundDis

#Tim
uint8[] timState

#safe touch
uint8 safe_touch

uint8[] fault_code
