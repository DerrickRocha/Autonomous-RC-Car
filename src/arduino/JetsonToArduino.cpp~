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


int main(){

	/* open serial port */
  	int fd = open("/dev/ttyACM0", O_RDWR | O_NOCTTY);
	char key;
	while(true){

	std::cout << "Press a key\n";
      	std::cin >> key;
      	std::cout << "You pressed: " << key;

	}
   	
      	
      
  	/*moveForward(fd);
	steerLeft(fd);
	steerRight(fd);
	steerStraight(fd);
	moveBackward(fd);*/
	
	return 0;

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
