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
#include "commands.h"

/*-------------------------------------------------------------
GLOBAL VARIABLES
---------------------------------------------------------------*/
double roll, pitch, yaw;
double roll_speed, pitch_speed, yaw_speed;
double global_lat, global_lon, global_alt;

int System_id;
int Pixhawk_comp_id;

int flag_offboard_control = 0;

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
    uint64_t time_command_ack = 0;

    mavlink_heartbeat_t heartbeat;
    mavlink_global_position_int_t global_position_int;
   	mavlink_attitude_t attitude;
    mavlink_param_value_t parameter;
    mavlink_command_ack_t command_ack;


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

                    case MAVLINK_MSG_ID_COMMAND_ACK:
                    {
                        //printf("MAVLINK_MSG_ID_ATTITUDE\n");
                        mavlink_msg_command_ack_decode(&message, &command_ack);
                        time_command_ack = get_time_usec();
                        //this_timestamps.attitude = current_messages.time_stamps.attitude;
                        break;
                    }

                    default:
                    {
                        // printf("Warning, did not handle message id %i\n",message.msgid);
                        break;
                    }


                } // end: switch msgid

                received_all = time_heartbeat && time_global_position_int && time_attitude; // && time_parameter;

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
    
    //printf("Parameter char %s is %f \n",parameter.param_id, parameter.param_value);
    //printf("Mav Type %d \n Mav Autopilot %d \n Mav Mode Flag %d \n Mav State %d \n",heartbeat.type,heartbeat.autopilot,heartbeat.base_mode,heartbeat.system_status);
    printf("Command %d was acknowledged with result %d \n",command_ack.command, command_ack.result);
    flag_offboard_control = command_ack.result;

    return 0;
}

int Pixhawk_write(int fd){
    mavlink_message_t message;
    char buf[300];

    enable_offboard_control(&message);
    //set_parameter(&message);
    // --------------------- Message Write ------------------------- //
    unsigned len = mavlink_msg_to_send_buffer((uint8_t*)buf, &message);

    const int bytesWritten = write(fd, buf, len);

}

main(){
    char *Pixhawk_uart_name = "/dev/ttyACM3";
    int Pixhawk_baudrate = 57600;

    int Pixhawk_port_number = Pixhawk_init(Pixhawk_uart_name,Pixhawk_baudrate);
    if(Pixhawk_port_number < 0){return 0;}

    Pixhawk_read(Pixhawk_port_number);
    printf("This is system %d with Pixhawk component number %d \n", System_id, Pixhawk_comp_id);

    while (flag_offboard_control != 1){
        Pixhawk_write(Pixhawk_port_number);
        Pixhawk_read(Pixhawk_port_number);
    }

    printf("Offboard Control Enabled Successfully \n");
    printf("Pressione qualquer tecla para iniciar \n");
    getch();

    

    while(!kbhit()){

    Pixhawk_write(Pixhawk_port_number);
    usleep(50000);

    Pixhawk_read(Pixhawk_port_number);
    printf("Roll Pitch Yaw Angles : %f  %f  %f (degree) \n", roll, pitch, yaw);
    printf("Roll Pitch Yaw Speeds : %f  %f  %f (degree/s) \n", roll_speed, pitch_speed, yaw_speed);
    printf("Latitude Longitude Altitude  : %f  %f  %f \n \n", global_lat, global_lon, global_alt);

    usleep(50000);
    }

    close(Pixhawk_port_number);

    return 0;
}