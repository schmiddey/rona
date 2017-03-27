Header header     

uint32 id         #id from frobot
uint32 priority   #prio from robot

float64 robotradius

geometry_msgs/PoseStamped robotPose
uint32 process
bool isStatic

nav_msgs/Path path
float64[] motion
float64[] motion_abs