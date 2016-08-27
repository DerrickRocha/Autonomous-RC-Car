#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <sys/ioctl.h>

#include "JetsonToArduino.h"


void main(){

	/* open serial port */
  	int fd = open("/dev/ttyACM0", O_RDWR | O_NOCTTY);
  	moveForward(fd);
	steerLeft(fd);
	steerRight(fd);
	steerStraight(fd);
	moveBackward(fd);


}


void moveForward(int fd){

	sendToArduino(fd,FORWARD);


}

void moveBackward(int fd){

	sendToArduino(fd,BACKWARD);

}

void steerLeft(int fd){

	sendToArduino(fd,LEFT);

}

void steerRight(int fd){

	sendToArduino(fd,RIGHT);

}

void steerStraight(int fd){

	sendToArduino(fd,STEER_STRAIGHT);

}


void sendToArduino(int fd, int direction){

	char tmp[12]={0x0};
	sprintf(tmp,"%11d", direction);
	write(fd, tmp, sizeof(tmp));

}
