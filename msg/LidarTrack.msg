# Position and velocity describe header.stamp (end of the carried scan sweep), in map.
std_msgs/Header header
uint32 id
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
float32 width
uint8 UNCHECKED=0
uint8 ENEMY=1
uint8 CHECKED_EMPTY=2
uint8 label
builtin_interfaces/Time last_return  # capture time of the latest supporting sweep
