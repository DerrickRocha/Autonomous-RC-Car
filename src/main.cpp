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
#include "Simulator.h"

#define WIDTH	320
#define HEIGHT	240
#define FPS	60
#define MAX_DEPTH 1.8
// Scale fator for adjusting the scale of the 3D reconstruction
#define SCALE_FACTOR_3D_RECONSTRUCTION 50.0
const string INSTALL_DIRECTORY = "/home/autonomousCar/";
const string DATA_DIRECTORY = "/home/" + string(getlogin()) + "/autonomousCar/"; // user home directory

int main(int argc, char **argv) {
	StereoPair stereoCam(WIDTH, HEIGHT, FPS, DATA_DIRECTORY);
    	stereoCam.maximumDepth = MAX_DEPTH;
	//stereoCam.calibrate();
       namedWindow("Controls", CV_WINDOW_NORMAL);
        int fd = open("/dev/ttyACM0", O_RDWR | O_NOCTTY);
	bool isRunning = false;
    while(true){

        int keyPressed = int(char(waitKey(1000)));
        if( keyPressed== 27) break;
        if(keyPressed == 'u'){
           stop(fd);
        }
          if(keyPressed == 255 && isRunning){
            stop(fd);
            steerStraight(fd);
            isRunning = false;
        }
	  if(keyPressed == 'i'){ 
            moveForward(fd);
  	    isRunning = true;
        
        }
	  if(keyPressed == 'j'){ 
            steerLeft(fd);  
  	    isRunning = true;  
          }
	  if(keyPressed == 'l'){
             steerRight(fd);
             isRunning = true;
          }
	  if(keyPressed == 'o'){
             steerStraight(fd);
             isRunning = true;
          }
	  if(keyPressed == ','){
	     moveBackward(fd); 
             isRunning = true;   
	  }  
          std::cout << "\nYou pressed: " << keyPressed;
    }

  
   
   
       // stereoCam.displayDisparityMap();
    float scenWidth = 2.0;  // meters
    float scenDepth = 1.0;  // meters
    float squareSize = 0.08; // meters
    int iteration = 0;
    float fieldOfView = -1;
    Simulator simulator = Simulator(scenWidth, scenDepth, squareSize, 1200, DATA_DIRECTORY);
    simulator.scenario.minY = 0.0;
    simulator.scenario.maxY = 0.1;
    simulator.scenario.scaleFactor = SCALE_FACTOR_3D_RECONSTRUCTION;
    isRunning = false;
	 while(true){
		////////////////Camera update////////////////////
		stereoCam.updateImages();
		stereoCam.updateDisparityImg();
		stereoCam.updateImage3D();
		if (iteration == 0) fieldOfView = stereoCam.computeFieldOfView();
		/////////////Obstacle map update/////////////////
		Mat image3D = stereoCam.image3D;
		bool obstaclesDetected = false;
		simulator.scenario.populateScenario(image3D, obstaclesDetected);
		///////////Display internal update///////////////
		Mat display = simulator.drawScenario(simulator.scenario.points, fieldOfView);
		//printf("fieldOfView = %f\n",fieldOfView);
		////////////Avoidance path update////////////////
		float newCurveRadius = 10000.0; // In meters. 10000 means infinity (straight path)
		if (obstaclesDetected) {

		    PathPlaner planer;
		    newCurveRadius = planer.findAvoidancePath(simulator.scenario, 10000, display, simulator.squarePixelSize);
		    printf("obstacles detected. Curve radius is %f.\n",newCurveRadius);
		    stop(fd);
		    isRunning = false;
		}
		else if(!isRunning){
		    printf("obstacles not detected. Curve radius is %f.\n",newCurveRadius);
		    moveForward(fd);
		    isRunning = true;
		}
		///////////////Display update////////////////////
		imshow("Simulator", display);

		int keyPressed = int(char(waitKey(10)));
		if( keyPressed== 27) {
		    destroyWindow("Simulator");
		    for(int i = 0; i < 10; i++) waitKey(1);
		    return 0;
		}
		if(keyPressed == 'd'){
	     		stereoCam.displayDisparityMap();  
	  	}
		iteration++;
	    }

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
