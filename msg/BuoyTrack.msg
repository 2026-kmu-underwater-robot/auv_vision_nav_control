uint8 CANDIDATE=0
uint8 CONFIRMED=1
uint8 ASSIGNED=2
uint8 APPROACHING=3
uint8 SERVICED=4
uint8 STALE=5
uint8 LOST=6
uint8 REJECTED=7

uint32 id
int32 class_id
geometry_msgs/Point position_mission
float32 position_rms_m
uint32 observation_count
uint8 status
builtin_interfaces/Time last_seen
