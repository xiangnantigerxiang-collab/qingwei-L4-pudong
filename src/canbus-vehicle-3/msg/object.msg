int16 id
uint8 type          # 0-car, 1-pedestrian, 2-cyclist, 3-unknown
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
