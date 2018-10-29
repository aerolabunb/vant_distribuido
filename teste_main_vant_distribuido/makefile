# gcc -Wall -D_REENTRANT -D_POSIX_TIMERS timer.c -o timer -lrt 
CFLAGS = -Wall -D_REENTRANT -D_POSIX_TIMERS -Wwrite-strings

TARGET = main
CC = g++
LIBS =  -lpthread -lrt -lm
INCLUDES =
OBJS =  ${TARGET}.o
SRCS =  ${TARGET}.c gdatalogger/gdatalogger.c gdatalogger/gmatlabdatafile.c  gdatalogger/gqueue.c system_init.c
HDRS =  

clean:
	rm ${TARGET} -f

all: clean ${TARGET}

# The variable $@ has the value of the target. In this case $@ = psort
${TARGET}:
	${CC} ${CFLAGS} ${INCLUDES}  ${SRCS} -o $@ ${LIBS}



