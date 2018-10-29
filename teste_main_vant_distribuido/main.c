#include <math.h>
#include <time.h>
#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <signal.h>
#include <sys/io.h>
#include <errno.h>
#include <sys/time.h>
#include <unistd.h>
#include <pthread.h>
#include <termios.h>
#include <fcntl.h>

#include "time.h"
#include "gdatalogger/gqueue.h"
#include "gdatalogger/gmatlabdatafile.h"
#include "gdatalogger/gdatalogger.h"
#include "system_init.h"

/*-------------------------------------------------------------
GLOBAL VARIABLES
---------------------------------------------------------------*/
double roll, pitch, yaw;
double roll_speed, pitch_speed, yaw_speed;
double global_lat, global_lon, global_alt;

int System_id;
int Pixhawk_comp_id;

uint64_t get_time_usec(){
	static struct timeval _time_stamp;
	gettimeofday(&_time_stamp, NULL);
	return _time_stamp.tv_sec*1000000 + _time_stamp.tv_usec;
}

int Pixhawk_read(int fd){

    mavlink_message_t message;
    uint8_t cp;
    mavlink_status_t status;
	uint8_t          msgReceived = false;
    bool received_all = false;  // receive only one message

    uint64_t time_heartbeat = 0;
    uint64_t time_global_position_int = 0;
	uint64_t time_attitude = 0;
    uint64_t time_parameter = 0;

    mavlink_heartbeat_t heartbeat;
    mavlink_global_position_int_t global_position_int;
   	mavlink_attitude_t attitude;
    mavlink_param_value_t parameter;


    while (!received_all)
	{
       
        int result = read(fd, &cp, 1);

        //printf("Result%d \n",result);
        msgReceived = mavlink_parse_char(MAVLINK_COMM_1, cp, &message, &status);

        int system_id = message.sysid;
        int component_id = message.compid;

        System_id = system_id;
        Pixhawk_comp_id = component_id;

        //printf("System id %d \n",system_id);
        //printf("Component id %d \n",component_id);



        switch (message.msgid)
                {

                    case MAVLINK_MSG_ID_HEARTBEAT:
                    {
                        //printf("MAVLINK_MSG_ID_HEARTBEAT\n");
                        mavlink_msg_heartbeat_decode(&message, &heartbeat);
                        time_heartbeat = get_time_usec();
                        //this_timestamps.heartbeat = current_messages.time_stamps.heartbeat;
                        break;
                    }

                    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
                    {
                        //printf("MAVLINK_MSG_ID_GLOBAL_POSITION_INT\n");
                        mavlink_msg_global_position_int_decode(&message, &global_position_int);
                        time_global_position_int = get_time_usec();
                        //this_timestamps.global_position_int = current_messages.time_stamps.global_position_int;
                        break;
                    }

                    case MAVLINK_MSG_ID_ATTITUDE:
                    {
                        //printf("MAVLINK_MSG_ID_ATTITUDE\n");
                        mavlink_msg_attitude_decode(&message, &attitude);
                        time_attitude = get_time_usec();
                        //this_timestamps.attitude = current_messages.time_stamps.attitude;
                        break;
                    }

                    case MAVLINK_MSG_ID_PARAM_VALUE:
                    {
                        //printf("MAVLINK_MSG_ID_ATTITUDE\n");
                        mavlink_msg_param_value_decode(&message, &parameter);
                        time_parameter = get_time_usec();
                        //this_timestamps.attitude = current_messages.time_stamps.attitude;
                        break;
                    }

                    default:
                    {
                        // printf("Warning, did not handle message id %i\n",message.msgid);
                        break;
                    }


                } // end: switch msgid

                received_all = time_heartbeat && time_global_position_int && time_attitude && time_parameter;

    }


    roll = (double)attitude.roll*180/PI;
    pitch = (double)attitude.pitch*180/PI;
    yaw = (double)attitude.yaw*180/PI;

    roll_speed = (double)attitude.rollspeed*180/PI;
    pitch_speed = (double)attitude.pitchspeed*180/PI;
    yaw_speed = (double)attitude.yawspeed*180/PI;

    global_lat = (double)global_position_int.lat*0.0000001;
    global_lon = (double)global_position_int.lon*0.0000001;
    global_alt = (double)global_position_int.alt*0.001;
    
    printf("Parameter char %s is %f \n",parameter.param_id, parameter.param_value);

    return 0;
}

int Pixhawk_write(int fd){
    mavlink_message_t message;
    char buf[300];
    char get;

    mavlink_rc_channels_override_t rc;
    mavlink_set_position_target_global_int_t target;
    mavlink_attitude_t attitude_set;
    mavlink_param_request_read_t parameters;
    mavlink_param_set_t parameter_set;

    /*rc.chan3_raw = 1600;
    rc.target_system = 1;
    rc.target_component;

    rc.chan1_raw = 0;
    rc.chan2_raw = 0;
    rc.chan4_raw = 0;
    rc.chan5_raw = 0;
    rc.chan6_raw = 0;
    rc.chan7_raw = 0;
    rc.chan8_raw = 0;
    rc.chan9_raw = 0;
    rc.chan10_raw = 0;
    rc.chan11_raw = 0;
    rc.chan12_raw = 0;
    rc.chan13_raw = 0;
    rc.chan14_raw = 0;
    rc.chan15_raw = 0;
    rc.chan16_raw = 0;
    rc.chan17_raw = 0;

    mavlink_msg_rc_channels_override_encode(1, 2, &message, &rc);
    
    target.vx = 10;
    target.vy = 0;
    target.vz = 0;
    target.target_system = 1;
    target.target_component = 1;
    target.coordinate_frame = MAV_FRAME_GLOBAL_INT;*/

    //mavlink_msg_set_position_target_global_int_encode(1, 1, &message, &target);

    /*attitude_set.roll = 0;
    attitude_set.pitch = 0;
    attitude_set.yaw = 0;

    mavlink_msg_attitude_encode(1, 1, &message, &attitude_set);*/

    parameters.target_system = 2;
    parameters.target_component = 1;
    parameters.param_index = -1;
    strncpy(parameters.param_id, "SYSID_THISMAV", sizeof(parameters.param_id));

    mavlink_msg_param_request_read_encode(1, 1, &message, &parameters);

    unsigned len = mavlink_msg_to_send_buffer((uint8_t*)buf, &message);

    const int bytesWritten = write(fd, buf, len);


    /*parameter_set.target_system = 2;
    parameter_set.target_component = 1;
    strncpy(parameter_set.param_id, "SYSID_THISMAV", sizeof(parameters.param_id));
    parameter_set.param_value = 1;

    mavlink_msg_param_set_encode(1, 1, &message, &parameter_set);

    unsigned len2 = mavlink_msg_to_send_buffer((uint8_t*)buf, &message);

    const int bytesWritten2 = write(fd, buf, len2);*/



}

main(){
    char *Pixhawk_uart_name = "/dev/ttyACM1";
    int Pixhawk_baudrate = 57600;

    int Pixhawk_port_number = Pixhawk_init(Pixhawk_uart_name,Pixhawk_baudrate);

    Pixhawk_write(Pixhawk_port_number);
    Pixhawk_read(Pixhawk_port_number);

    printf("This is sytem %d with Pixhawk component number %d \n", System_id, Pixhawk_comp_id);
    printf("Pressione qualquer tecla para iniciar \n");
    getch();

    

    while(1){

    Pixhawk_write(Pixhawk_port_number);

    Pixhawk_read(Pixhawk_port_number);
    printf("Roll Pitch Yaw Angles : %f  %f  %f (degree) \n", roll, pitch, yaw);
    printf("Roll Pitch Yaw Speeds : %f  %f  %f (degree/s) \n", roll_speed, pitch_speed, yaw_speed);
    printf("Latitude Longitude Altitude  : %d  %d  %d \n \n", global_lat, global_lon, global_alt);

    usleep(100000);
    }

    

    return 0;
}