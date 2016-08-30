#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <iostream>
#include <stdlib.h>

#include "JetsonToArduino.h"


void moveForward(int fd){

	sendToArduino(fd,FORWARD);


}

void moveBackward(int fd){

	sendToArduino(fd,BACKWARD);

}

void stop(int fd){
  
  sendToArduino(fd,STOP);
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
	printf("moving %d",direction);
	char tmp[12]={0x0};
	sprintf(tmp,"%11d", direction);
	write(fd, tmp, sizeof(tmp));

}
