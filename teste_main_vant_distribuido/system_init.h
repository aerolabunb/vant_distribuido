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

#include "c_library_v1/common/mavlink.h"


#define PIXHAWK_TASK_PERIOD_US          200000      // sample period of Pixhawk read/write thread (us)
#define MODEM_TASK_PERIOD_US            200000      // sample period of Modem read/write thread (us)
#define PI                              3.14159265
#define MODEM_READ_ENABLE               0
#define MODEM_WRITE_ENABLE              0
#define PIXHAWK_READ_ENABLE             1
#define PIXHAWK_WRITE_ENABLE            1


int open_port(const char* port);
bool setup_port(int port_number, int baud, int data_bits, int stop_bits, bool parity, bool hardware_control);
int Pixhawk_init(const char* Pixhawk_uart_name, int Pixhawk_baudrate);
int getch();
int kbhit(void);