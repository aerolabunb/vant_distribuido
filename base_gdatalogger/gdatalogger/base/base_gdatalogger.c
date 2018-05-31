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

#include "../gqueue.h"
#include "../gmatlabdatafile.h"
#include "../gdatalogger.h"
#include <stdio.h>

GDATALOGGER gDataLogger;

int main(){
	double t = 1;

//----------------------------Data logger-----------------------------------//
	if(!gDataLogger_Init(&gDataLogger,(char*) "matlabdatafiles/gmatlabdatafile.mat",NULL)){
		printf("\nErro em gDataLogger_Init\n\n");
		return 0;
	}

	gDataLogger_DeclareVariable(&gDataLogger,(char*) "T",(char*) "s",1,1,1000);
	gDataLogger_InsertVariable(&gDataLogger,(char*) "T",&t);
	gDataLogger_IPCUpdate(&gDataLogger); // gerencia IPC
        //gDataLogger_MatfileUpdate(&gDataLogger); // esvazia os buffers no arquivo de log
	gDataLogger_Close(&gDataLogger);


return 0;
}
