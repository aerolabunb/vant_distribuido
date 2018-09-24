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
SERIAL AND AUTOPILOT
---------------------------------------------------------------*/
//Autopilot_Interface *autopilot_interface_quit;
//Serial_Port *serial_port_quit;
//void quit_handler( int sig );



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
    //commands(autopilot_interface);
}


/*-------------------------------------------------------------
MODEM COMMUNICATION THREAD
---------------------------------------------------------------*/
void comm_Modem(union sigval arg){
    
    printf("Thread Modem \n");

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
	
	timer_start_Pixhawk ();
    timer_start_Modem ();
    printf("Thread main iniciada \n");
    while(1){
        Pixhawk_Init();
        //init_Autopilot();
        gDataLogger_IPCUpdate(&gDataLogger); // gerencia IPC
    }
	time_reset(&timestruct_Pixhawk);
    time_reset(&timestruct_Modem);
    gDataLogger_Close(&gDataLogger);
	timer_stop_Pixhawk ();
    timer_stop_Modem ();

return 0;
}
