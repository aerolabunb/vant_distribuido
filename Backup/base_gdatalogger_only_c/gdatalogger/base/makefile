# gcc -Wall -D_REENTRANT -D_POSIX_TIMERS timer.c -o timer -lrt 
CFLAGS = -Wall -D_REENTRANT -D_POSIX_TIMERS -Wwrite-strings

TARGET = base_gdatalogger
CC = g++
LIBS =  -lpthread -lrt -lm
INCLUDES =
OBJS =  ${TARGET}.o
SRCS =  ${TARGET}.c ../gdatalogger.c ../gmatlabdatafile.c  ../gqueue.c mavlink/mavlink_control.cpp mavlink/serial_port.cpp mavlink/autopilot_interface.cpp
HDRS =  

clean:
	rm ${TARGET} -f

all: clean ${TARGET}

# The variable $@ has the value of the target. In this case $@ = psort
${TARGET}:
	${CC} ${CFLAGS} ${INCLUDES}  ${SRCS} -o $@ ${LIBS}



