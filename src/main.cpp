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
#include <boost/filesystem.hpp>
#include "arduino/JetsonToArduino.h"
#include "StereoPair.h"

#define WIDTH	320
#define HEIGHT	240
#define FPS	60
#define MAX_DEPTH 1.8
const string INSTALL_DIRECTORY = "/home/autonomousCar/";
const string DATA_DIRECTORY = "/home/" + string(getlogin()) + "/autonomousCar/"; // user home directory


int main(int argc, char **argv) {
	StereoPair stereoCam(WIDTH, HEIGHT, FPS, DATA_DIRECTORY);
    	stereoCam.maximumDepth = MAX_DEPTH;
        stereoCam.displayDisparityMap();

   //open serial port 
  	/*int fd = open("/dev/ttyACM0", O_RDWR | O_NOCTTY);
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
		

	}*/
   	
	
	return 0;
}
