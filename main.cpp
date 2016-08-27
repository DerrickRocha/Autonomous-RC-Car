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

#include "arduino/JetsonToArduino.h"

int main(int argc, char **argv) {
  /* open serial port */
  	int fd = open("/dev/ttyACM0", O_RDWR | O_NOCTTY);
	char key;
	while(true){
	  std::cout << "\nPress a key\n";
	  std::cin >> key;
	  std::cout << "\nYou pressed: " << key;
	  if(key == 'u') stop(fd);
	  if(key == 'i') moveForward(fd);
	  if(key == 'j') steerLeft(fd);
	  if(key == 'l') steerRight(fd);
	  if(key == 'o') steerStraight(fd);
	  if(key == ',') moveBackward(fd);

	}
   	
      	
      
  	/*moveForward(fd);
	steerLeft(fd);
	steerRight(fd);
	steerStraight(fd);
	moveBackward(fd);*/
	
	return 0;
}
