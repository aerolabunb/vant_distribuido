#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

int configUART1(){
	struct termios cUART1;
<<<<<<< HEAD
	int UART1 = open("/dev/ttyUSB1",O_RDWR);
	if(tcgetattr(UART1,&cUART1))printf("Erro tcgetattr");
	cfmakeraw(&cUART1);
	cfsetspeed(&cUART1,B9600);
=======
	int UART1 = open("/dev/ttyO0",O_RDWR);
	if(tcgetattr(UART1,&cUART1))printf("Erro tcgetattr");
	cfmakeraw(&cUART1);
	cfsetspeed(&cUART1,B115200);
>>>>>>> 80f98fc4bb8d1c1a609ddb499e04b465a4009083
	cUART1.c_cflag &= ~CSTOPB;
	cUART1.c_cc[VMIN] = 1;
	cUART1.c_cc[VTIME] = 1;
	if(tcsetattr(UART1,TCSAFLUSH,&cUART1))printf("Erro tcsetattr");
	return UART1;
}

int main(){
	int UART1 = configUART1();
	char dis[2], out[100],string[100];
	printf("Que dispositivo eu sou?");
	scanf("%c",&dis[0]);
	dis[1]=0;
	string[0]=0;
	strcat(string,"Ola! Essa é uma mensagem do dispositivo ");
	strcat(string,dis);

	//testa UART
	write(UART1,string,strlen(string));
	sleep(5);
	read(UART1,out,100);
	
	printf("Mensagem lida pelo dispositivo %s: %s\n",dis,out);	

	close(UART1);

	return 0;
<<<<<<< HEAD
}
=======
}
>>>>>>> 80f98fc4bb8d1c1a609ddb499e04b465a4009083
