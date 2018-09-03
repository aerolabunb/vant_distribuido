#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>

int configUART1(){
	struct termios cUART1;
	int UART1 = open("/dev/ttyO2",O_RDWR);
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

int main(){
	int UART1 = configUART1();
	char dis[2], out[100],string[100];
    	int n = 0, i = 0;
	printf("Unidade Coordenadora \nFuncao Transmissora \n");
	//scanf("%c",&dis[0]);
	//dis[1]=0;
	string[0]=0;
	strcat(string,"bruno ");
	//strcat(string,dis);

	//testa UART
    while(i<10000){
	n = write(UART1,string,strlen(string)); //Transmite a mensagem
	//printf("%d Bytes enviados \n",n);
    	sleep(0.1);
	i++;
    }
	close(UART1);

	return 0;
}
