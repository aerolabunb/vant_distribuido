#include "system_init.h"
#include "c_library_v1/common/mavlink.h"

int enable_offboard_control(mavlink_message_t* message);
int disable_offboard_control(mavlink_message_t* message);

int command_int(mavlink_message_t* message);
int actuator_control(mavlink_message_t* message);
int rc_command(mavlink_message_t* message);
int set_global_int(mavlink_message_t* message);
int request_parameter(mavlink_message_t* message);
int set_parameter(mavlink_message_t* message);
int set_local_ned(mavlink_message_t* message);