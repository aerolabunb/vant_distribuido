#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>

int configUART1(){
	struct termios cUART1;
	int UART1 = open("/dev/ttyUSB1",O_RDWR| O_NOCTTY | O_NONBLOCK);
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
    int n = 0;
	printf("Unidade Remota inicializada com sucesso \nFuncao Receptora \n \n");
	//scanf("%c",&dis[0]);
	//dis[1]=0;
	//string[0]=0;
	//strcat(string,"A");
	//strcat(string,dis);

	//testa UART
    while(1){
    printf("Buscando mensagens \n");
	n = read(UART1,&out,5);
    if(n<0){
        printf("Erro durante leitura %d\n",(int)errno);
   		}
    else{
		printf("Mensagem recebida: %s\n",out);
    	printf("Numero de bytes lidos: %d \n \n",n);
    	}
    sleep(1);
    }
	close(UART1);

	return 0;
}
