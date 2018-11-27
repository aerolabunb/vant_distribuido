#include "commands.h"

int enable_offboard_control(mavlink_message_t* message){

	mavlink_command_long_t com = { 0 };
	com.target_system    = 1;
	com.target_component = 1;
	com.command          = MAV_CMD_NAV_GUIDED_ENABLE;
	com.confirmation     = true;
	com.param1           = (float) true; // flag >0.5 => start, <0.5 => stop

	mavlink_msg_command_long_encode(1, 1, message, &com);

    return 0;
}
 

int command_int(mavlink_message_t* message){
        
    mavlink_command_int_t command;

    command.target_system = 1;
    command.target_component = 1;
    command.command = 178;
    //command.param1 = 0;
    command.param2 = 20;
    command.frame = MAV_FRAME_LOCAL_NED;
    command.current = 1;

    mavlink_msg_command_int_encode(1, 1, message, &command);

    return 0;
}

int actuator_control(mavlink_message_t* message){
    mavlink_set_actuator_control_target_t control;

    control.controls[3] = 1;
    control.target_system = 1;
    control.target_component = 1;

    mavlink_msg_set_actuator_control_target_encode(1, 1, message, &control);

    return 0;
    }

int rc_command(mavlink_message_t* message){
    // ------------------- RC Channel Override function -------------------- //
    mavlink_rc_channels_override_t rc;
    rc.chan3_raw = 1600;
    rc.target_system = 1;
    rc.target_component = 1;

    rc.chan1_raw = 0;
    rc.chan2_raw = 0;
    rc.chan4_raw = 0;
    rc.chan5_raw = 0;
    rc.chan6_raw = 0;
    rc.chan7_raw = 0;
    rc.chan8_raw = 0;

    mavlink_msg_rc_channels_override_encode(1, 1, message, &rc);
    
    return 0;

}

int set_global_int(mavlink_message_t* message){
    // ------------------- Global set function -------------------- //
    mavlink_set_position_target_global_int_t target;

    target.vx = 10;
    target.vy = 0;
    target.vz = 0;
    target.target_system = 1;
    target.target_component = 1;
    target.coordinate_frame = MAV_FRAME_GLOBAL_INT;

    mavlink_msg_set_position_target_global_int_encode(1, 1, message, &target);

    return 0;

}
int request_parameter(mavlink_message_t* message){
    // ------------------- Request Parameter function -------------------- //
    mavlink_param_request_read_t parameters;
    parameters.target_system = 1;
    parameters.target_component = 1;
    parameters.param_index = -1;
    strncpy(parameters.param_id, "SYSID_THISMAV", sizeof(parameters.param_id));

    mavlink_msg_param_request_read_encode(1, 1, message, &parameters);

    return 0;
    
}

int set_parameter(mavlink_message_t* message){
    // ------------------- Set Parameter function -------------------- //
    mavlink_param_set_t parameter_set;

    parameter_set.target_system = 2;
    parameter_set.target_component = 1;
    strncpy(parameter_set.param_id, "SYSID_THISMAV", sizeof(parameter_set.param_id));
    parameter_set.param_value = 1;

    mavlink_msg_param_set_encode(1, 1, message, &parameter_set);

    return 0;
}

int set_local_ned(mavlink_message_t* message){
    // ------------------- Local set function -------------------- //
    mavlink_set_position_target_local_ned_t local_target;

    local_target.vx = 10;
    local_target.vy = 0;
    local_target.vz = 0;
    local_target.target_system = 1;
    local_target.target_component = 1;
    local_target.coordinate_frame = MAV_FRAME_LOCAL_NED;

    mavlink_msg_set_position_target_local_ned_encode(1, 1, message, &local_target);

    return 0;
    }