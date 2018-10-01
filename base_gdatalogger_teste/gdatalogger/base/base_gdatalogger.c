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
#include <stdio.h>

#define TASK_PERIOD_US                  200000
#define PI                              3.14159265

GDATALOGGER gDataLogger;

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

/*-------------------------------------------------------------
SERIAL AND AUTOPILOT CONFIGURATIONS (PIXHAWK)
---------------------------------------------------------------*/
Autopilot_Interface *autopilot_interface_quit;
Serial_Port *serial_port_quit;
void quit_handler( int sig );

char *global_uart_name = (char*)"/dev/ttyACM0";
int global_baudrate = 57600;

Serial_Port global_serial_port(global_uart_name, global_baudrate);
Autopilot_Interface global_autopilot_interface(&global_serial_port);

/*-------------------------------------------------------------
TIMER FUNCTIONS
---------------------------------------------------------------*/
void timer_start_Pixhawk (void)
{
    struct itimerspec itimer = { { 1, 0 }, { 1, 0 } };
    struct sigevent sigev;

    itimer.it_interval.tv_sec=0;
    itimer.it_interval.tv_nsec=TASK_PERIOD_US * 1000;
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
    itimer.it_interval.tv_nsec=TASK_PERIOD_US * 1000;
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

/*-------------------------------------------------------------
PIXHAWK COMMUNICATION THREAD
---------------------------------------------------------------*/
void comm_Pixhawk(union sigval arg){

    printf("Thread Pixhawk \n");
    commands(global_autopilot_interface);
}


/*-------------------------------------------------------------
MODEM COMMUNICATION THREAD
---------------------------------------------------------------*/
void comm_Modem(union sigval arg){
    
    printf("Thread Modem \n");

}

// ------------------------------------------------------------------------------
//   Quit Signal Handler
// ------------------------------------------------------------------------------
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


// ------------------------------------------------------------------------------
//   PORT and THREAD STARTUP
// ------------------------------------------------------------------------------
int
top ()
{

	serial_port_quit         = &global_serial_port;
	autopilot_interface_quit = &global_autopilot_interface;
	signal(SIGINT,quit_handler);

	/*
	 * Start the port and autopilot_interface
	 * This is where the port is opened, and read and write threads are started.
	 */
	global_serial_port.start();
	global_autopilot_interface.start();


	return 0;

}


/*-------------------------------------------------------------
MAIN FUNCTION
---------------------------------------------------------------*/
int main(){
	double t = 1;

//----------------------------Data logger-----------------------------------//
	if(!gDataLogger_Init(&gDataLogger,(char*) "matlabdatafiles/gmatlabdatafile.mat",NULL)){
		printf("\nErro em gDataLogger_Init\n\n");
		return 0;
	}

	gDataLogger_DeclareVariable(&gDataLogger,(char*) "T",(char*) "s",1,1,1000);
	gDataLogger_InsertVariable(&gDataLogger,(char*) "T",&t);
    //gDataLogger_MatfileUpdate(&gDataLogger); // esvazia os buffers no arquivo de log
	
//------------------------ Initialization -------------------------------------//    
    Pixhawk_Init();
    timer_start_Pixhawk ();
    timer_start_Modem ();
    printf("Thread main iniciada \n");

//----------------------- (Eternal) Loop ---------------------------------------//
    while(1){
        //init_Autopilot();
        gDataLogger_IPCUpdate(&gDataLogger); // gerencia IPC
    }
	time_reset(&timestruct_Pixhawk);
    time_reset(&timestruct_Modem);
    gDataLogger_Close(&gDataLogger);
	timer_stop_Pixhawk ();
    timer_stop_Modem ();

    // --------------------------------------------------------------------------
	//   THREAD and PORT SHUTDOWN
	// --------------------------------------------------------------------------

	/*
	 * Now that we are done we can stop the threads and close the port
	 */
	global_autopilot_interface.stop();
	global_serial_port.stop();

return 0;
}
