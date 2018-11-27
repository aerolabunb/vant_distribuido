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
#include "../gqueue.h"
#include "../gmatlabdatafile.h"
#include "../gdatalogger.h"
#include "mavlink/mavlink_control.h"
#include "mavlink/serial_port.h"
#include "mavlink/autopilot_interface.h"

#define PIXHAWK_TASK_PERIOD_US          200000      // sample period of Pixhawk read/write thread (us)
#define MODEM_TASK_PERIOD_US            200000      // sample period of Modem read/write thread (us)
#define PI                              3.14159265
#define MODEM_READ_ENABLE               0
#define MODEM_WRITE_ENABLE              0
#define PIXHAWK_READ_ENABLE             1
#define PIXHAWK_WRITE_ENABLE            1

GDATALOGGER gDataLogger;

/*-------------------------------------------------------------
GLOBAL VARIABLES
---------------------------------------------------------------*/
double roll, pitch, yaw;
double roll_speed, pitch_speed, yaw_speed;
double global_lat, global_lon, global_alt;


/*-------------------------------------------------------------
TIMER VARIABLES
---------------------------------------------------------------*/

int timer_nr_Pixhawk;
timer_t timer_Pixhawk;
void timer_start_Pixhawk (void);
void timer_stop_Pixhawk (void);
void comm_Pixhawk (union sigval sigval);

int timer_nr_Modem;
timer_t timer_Modem;
void timer_start_Modem (void);
void timer_stop_Modem (void);
void comm_Modem (union sigval sigval);
    
static timestruct_t timestruct_Pixhawk;
static timestruct_t timestruct_Modem;

int global_counter;

/*-------------------------------------------------------------
SERIAL AND AUTOPILOT CONFIGURATIONS (PIXHAWK)
---------------------------------------------------------------*/
Autopilot_Interface *autopilot_interface_quit;
Serial_Port *serial_port_quit;
void quit_handler( int sig );

char *Pixhawk_uart_name = (char*)"/dev/ttyACM4";
int Pixhawk_baudrate = 57600;

mavlink_highres_imu_t global_imu;
mavlink_local_position_ned_t global_pos;
mavlink_attitude_t global_attitude;
mavlink_global_position_int_t global_global_position;
mavlink_gps_status_t global_gps_status;
mavlink_set_position_target_local_ned_t global_sp;
mavlink_rc_channels_override_t global_rc;

/*-------------------------------------------------------------
MODEM CONFIGURATIONS
---------------------------------------------------------------*/
char *Modem_uart_name = (char*)"/dev/ttyUSB0";
int Modem_uart_port;


/*-------------------------------------------------------------
TIMER FUNCTIONS
---------------------------------------------------------------*/
void timer_start_Pixhawk (void)
{
    struct itimerspec itimer = { { 1, 0 }, { 1, 0 } };
    struct sigevent sigev;

    itimer.it_interval.tv_sec=0;
    itimer.it_interval.tv_nsec=PIXHAWK_TASK_PERIOD_US * 1000 ;
    itimer.it_value=itimer.it_interval;

    memset (&sigev, 0, sizeof (struct sigevent));
    sigev.sigev_value.sival_int = timer_nr_Pixhawk;
    sigev.sigev_notify = SIGEV_THREAD;
    sigev.sigev_notify_attributes = NULL;
    sigev.sigev_notify_function = comm_Pixhawk;

//    if (timer_create (CLOCK_MONOTONIC, &sigev, &timer) < 0)
    if (timer_create (CLOCK_REALTIME, &sigev, &timer_Pixhawk) < 0)
    {
        fprintf (stderr, "[%d]: %s\n", __LINE__, strerror (errno));
        exit (errno);
    }

    if (timer_settime (timer_Pixhawk, 0, &itimer, NULL) < 0)
    {
        fprintf (stderr, "[%d]: %s\n", __LINE__, strerror (errno));
        exit (errno);
    }
}

void timer_stop_Pixhawk (void)
{
    if (timer_delete (timer_Pixhawk) < 0)
    {
        fprintf (stderr, "[%d]: %s\n", __LINE__, strerror (errno));
        exit (errno);
    }
}

void timer_start_Modem (void)
{
    struct itimerspec itimer = { { 1, 0 }, { 1, 0 } };
    struct sigevent sigev;

    itimer.it_interval.tv_sec=0;
    itimer.it_interval.tv_nsec=MODEM_TASK_PERIOD_US * 1000;
    itimer.it_value=itimer.it_interval;

    memset (&sigev, 0, sizeof (struct sigevent));
    sigev.sigev_value.sival_int = timer_nr_Modem;
    sigev.sigev_notify = SIGEV_THREAD;
    sigev.sigev_notify_attributes = NULL;
    sigev.sigev_notify_function = comm_Modem;

//    if (timer_create (CLOCK_MONOTONIC, &sigev, &timer) < 0)
    if (timer_create (CLOCK_REALTIME, &sigev, &timer_Modem) < 0)
    {
        fprintf (stderr, "[%d]: %s\n", __LINE__, strerror (errno));
        exit (errno);
    }

    if (timer_settime (timer_Modem, 0, &itimer, NULL) < 0)
    {
        fprintf (stderr, "[%d]: %s\n", __LINE__, strerror (errno));
        exit (errno);
    }
}

void timer_stop_Modem (void)
{
    if (timer_delete (timer_Modem) < 0)
    {
        fprintf (stderr, "[%d]: %s\n", __LINE__, strerror (errno));
        exit (errno);
    }
}

// --------------------------------------------------------------
//   PIXHAWK READ UPDATE
// --------------------------------------------------------------

void
commands(Autopilot_Interface &api, Mavlink_Messages &messages)
{
	// copy current messages
	messages = api.current_messages;

	// local position in ned frame
	mavlink_local_position_ned_t pos = messages.local_position_ned;
	global_pos = pos;

	// hires imu
	mavlink_highres_imu_t imu = messages.highres_imu;
	global_imu = imu;

    mavlink_attitude_t attitude = messages.attitude;
	global_attitude = attitude;

    mavlink_global_position_int_t global_position = messages.global_position_int;
    global_global_position = global_position;

    mavlink_gps_status_t gps_status = messages.gps_status;
    global_gps_status = gps_status;

    #if PIXHAWK_WRITE_ENABLE
    //api.update_setpoint(global_sp);
    api.update_rc(global_rc);
    #endif
	return;

}

/*-------------------------------------------------------------
PIXHAWK COMMUNICATION THREAD
---------------------------------------------------------------*/
void comm_Pixhawk(union sigval arg){

    printf("Thread Pixhawk \n");

    #if PIXHAWK_READ_ENABLE

    roll = (double)global_attitude.roll*180/PI;
    pitch = (double)global_attitude.pitch*180/PI;
    yaw = (double)global_attitude.yaw*180/PI;

    roll_speed = (double)global_attitude.rollspeed*180/PI;
    pitch_speed = (double)global_attitude.pitchspeed*180/PI;
    yaw_speed = (double)global_attitude.yawspeed*180/PI;

    global_lat = (double)global_global_position.lat*0.0000001;
    global_lon = (double)global_global_position.lon*0.0000001;
    global_alt = (double)global_global_position.alt*0.001;

    printf("Local Position NED x, y, z :  %f %f %f (m)\n", global_pos.x, global_pos.y, global_pos.z );
    printf("Roll Pitch Yaw Angles : %f  %f  %f (degree) \n", roll, pitch, yaw);
    printf("Roll Pitch Yaw Speeds : %f  %f  %f (degree/s) \n", roll_speed, pitch_speed, yaw_speed);
    printf("Latitude Longitude Altitude  : %d  %d  %d \n", global_lat, global_lon, global_alt);
    //printf("Latitude Longitude Altitude Speeds  : %d  %d  %d \n", global_global_position.vx, global_global_position.vy, global_global_position.vz);

    gDataLogger_InsertVariable(&gDataLogger,(char*) "pitch",&pitch);
    gDataLogger_InsertVariable(&gDataLogger,(char*) "roll",&roll);
    gDataLogger_InsertVariable(&gDataLogger,(char*) "yaw",&yaw);

    gDataLogger_InsertVariable(&gDataLogger,(char*) "pitch_speed",&pitch_speed);
    gDataLogger_InsertVariable(&gDataLogger,(char*) "roll_speed",&roll_speed);
    gDataLogger_InsertVariable(&gDataLogger,(char*) "yaw_speed",&yaw_speed);

    gDataLogger_InsertVariable(&gDataLogger,(char*) "latitude",&global_lat);
    gDataLogger_InsertVariable(&gDataLogger,(char*) "longitude",&global_lon);
    gDataLogger_InsertVariable(&gDataLogger,(char*) "altitude",&global_alt);

	printf("\n");
    #endif

    #if PIXHAWK_WRITE_ENABLE
    // Insert here the command protocol

	//usleep(100); // give some time to let it sink in

	// now the autopilot is accepting setpoint commands
	printf("SEND OFFBOARD COMMANDS\n");

	// initialize command data strtuctures
	//mavlink_set_position_target_local_ned_t sp;
	//mavlink_set_position_target_local_ned_t ip = api.initial_position;

	// autopilot_interface.h provides some helper functions to build the command


	// Example 1 - Set Velocity
	//set_velocity(  10.0       , // [m/s]
	//			   0.0       , // [m/s]
	//			   0.0       , // [m/s]
	//			   global_sp        );

	// Example 2 - Set Position
//	 set_position( ip.x - 5.0 , // [m]
//			 	   ip.y - 5.0 , // [m]
//				   ip.z       , // [m]
//				   sp         );


	// Example 1.2 - Append Yaw Command
	//set_yaw( PI/8 , // [rad]
	//		 global_sp     );

	// SEND THE COMMAND
	// NOW pixhawk will try to move

    global_rc.chan3_raw = 1080;
    global_rc.target_system = 1;
    global_rc.target_component = 1;

	// Wait for 8 seconds, check position
	for (int i=0; i < 1; i++)
	{
		//mavlink_local_position_ned_t pos = api.current_messages.local_position_ned;
		//printf("%i CURRENT POSITION XYZ = [ % .4f , % .4f , % .4f ] \n", i, pos.x, pos.y, pos.z);
		sleep(1);
	}

	printf("\n");

    #endif

    global_counter++;
}

/*-------------------------------------------------------------
MODEM CONFIGURATION
---------------------------------------------------------------*/
int Modem_config(){
    struct termios cUART1;
	int UART1 = open(Modem_uart_name,O_RDWR| O_NOCTTY | O_NONBLOCK);
    //memset (&cUART1, 0, sizeof cUART1);
	if(tcgetattr(UART1,&cUART1)!=0){
        printf("Erro %d from tcgetattr \n",(int)errno);
    }
    cfmakeraw(&cUART1);
	cfsetspeed(&cUART1,B115200);
	cUART1.c_cflag &= ~CSTOPB;
	cUART1.c_cc[VMIN] = 1;
	cUART1.c_cc[VTIME] = 1;
	if(tcsetattr(UART1,TCSAFLUSH,&cUART1)!= 0){
        printf("Erro from tcsetattr \n");
    }
    return UART1;
}

/*-------------------------------------------------------------
MODEM COMMUNICATION THREAD
---------------------------------------------------------------*/
void comm_Modem(union sigval arg){
    
    printf("Thread Modem \n");
    char string[100], out[100];
    int n = 0, i = 0;
	//scanf("%c",&dis[0]);
	//dis[1]=0;
	string[0]=0;
	strcat(string,"bruno ");
	//strcat(string,dis);

	#if MODEM_WRITE_ENABLE
    while(i<10000){
	    n = write(Modem_uart_port,string,strlen(string)); //Transmite a mensagem
	    //printf("%d Bytes enviados \n",n);
    	sleep(0.1);
	    i++;
    }
    i = 0;
    #endif

    #if MODEM_READ_ENABLE
    while(i<10000){
    	printf("Buscando mensagens \n");
		n = read(Modem_uart_port,out,100);
    	if(n<0){
        	printf("Erro durante leitura \n");
    	}
    else{
		printf("Mensagem recebida: %s\n",out);
    	printf("Numero de bytes lidos: %d \n \n",n);
    }
    sleep(1);
    }
    #endif

}

// --------------------------------------------------------------
//   Quit Signal Handler
// --------------------------------------------------------------
// this function is called when you press Ctrl-C
void
quit_handler( int sig )
{
	printf("\n");
	printf("TERMINATING AT USER REQUEST\n");
	printf("\n");

	// autopilot interface
	try {
		autopilot_interface_quit->handle_quit(sig);
	}
	catch (int error){}

	// serial port
	try {
		serial_port_quit->handle_quit(sig);
	}
	catch (int error){}

	// end program here
	exit(0);

}

/*-------------------------------------------------------------
MAIN FUNCTION
---------------------------------------------------------------*/
int main(){
	double t = 1;
    int i = 0;
	Mavlink_Messages messages;

//----------------------------Data logger Initialization -----------------------------------//
	if(!gDataLogger_Init(&gDataLogger,(char*) "matlabdatafiles/gmatlabdatafile.mat",NULL)){
		printf("\nErro em gDataLogger_Init\n\n");
		return 0;
	}

	gDataLogger_DeclareVariable(&gDataLogger,(char*) "T",(char*) "s",1,1,1000);
    gDataLogger_DeclareVariable(&gDataLogger,(char*) "pitch",(char*) "deg",1,1,1000);
    gDataLogger_DeclareVariable(&gDataLogger,(char*) "roll",(char*) "deg",1,1,1000);
    gDataLogger_DeclareVariable(&gDataLogger,(char*) "yaw",(char*) "deg",1,1,1000);
    gDataLogger_DeclareVariable(&gDataLogger,(char*) "pitch_speed",(char*) "deg/s",1,1,1000);
    gDataLogger_DeclareVariable(&gDataLogger,(char*) "roll_speed",(char*) "deg/s",1,1,1000);
    gDataLogger_DeclareVariable(&gDataLogger,(char*) "yaw_speed",(char*) "deg/s",1,1,1000);
    gDataLogger_DeclareVariable(&gDataLogger,(char*) "latitude",(char*) "deg",1,1,1000);
    gDataLogger_DeclareVariable(&gDataLogger,(char*) "longitude",(char*) "deg",1,1,1000);
    gDataLogger_DeclareVariable(&gDataLogger,(char*) "altitude",(char*) "m",1,1,1000);

	
//------------------------ Pixhawk Initialization -------------------------------------//
    #if PIXHAWK_WRITE_ENABLE || PIXHAWK_READ_ENABLE
	Serial_Port serial_port(Pixhawk_uart_name, Pixhawk_baudrate);
	Autopilot_Interface autopilot_interface(&serial_port);

	// Setup interrupt signal handler 
	serial_port_quit         = &serial_port;
	autopilot_interface_quit = &autopilot_interface;
	signal(SIGINT,quit_handler);

	serial_port.start();
	autopilot_interface.start();

	commands(autopilot_interface, messages);
    timer_start_Pixhawk ();
    global_counter = 0;
    #else
    global_counter = 101;
    #endif

    #if PIXHAWK_WRITE_ENABLE
	autopilot_interface.enable_offboard_control();
    #endif

//------------------------ Modem Initialization --------------------------------//
    #if MODEM_WRITE_ENABLE || MODEM_READ_ENABLE
    Modem_uart_port = Modem_config();
    if(Modem_uart_port>0){printf("Modem Conectado com Sucesso \n");}
    timer_start_Modem ();
    #endif

//-----------------------  Loop ---------------------------------------//
    while(global_counter<100){               // Definir posteriormente a condicao de parada
		commands(autopilot_interface, messages);        // Atualiza a leitura e escrita das variáveis da Pixhawk
        gDataLogger_MatfileUpdate(&gDataLogger); // esvazia os buffers no arquivo de log
        gDataLogger_IPCUpdate(&gDataLogger); // gerencia IPC
    }

// --------------------- Close and Stop ports --------------------------------- //

	//time_reset(&timestruct_Pixhawk);
    //time_reset(&timestruct_Modem);
    gDataLogger_Close(&gDataLogger);
	timer_stop_Pixhawk ();
    timer_stop_Modem ();
    close(Modem_uart_port);

    #if PIXHAWK_WRITE_ENABLE
	autopilot_interface.disable_offboard_control();
    #endif

	autopilot_interface.stop();
	serial_port.stop();

return 0;
}
