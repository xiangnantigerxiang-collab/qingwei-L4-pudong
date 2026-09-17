int16 id
uint8 type          # HDMap 车道关系：0 本车道，1 左一，2 左二，3 左侧车道外，4 右侧车道外
float32 x           # x position in map
float32 y           # y position in map
float32 dx          # length, size in x direction of car frame
float32 dy          # width
float32 heading     # box rotation heading in map frame
float32 height      # box height
float32 vx          # velocity x in map frame
float32 vy          # velocity y in map frame
float32 confidence  # confidence for obstacle filter
geometry_msgs/Point[] polygons # polygons of objects
