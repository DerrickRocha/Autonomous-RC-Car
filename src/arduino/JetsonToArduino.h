

const int STOP = 0;
const int FORWARD = 1;
const int BACKWARD = 2;
const int LEFT = 3;
const int RIGHT = 4;
const int STEER_STRAIGHT = 5;


void stop(int fd);

void moveForward(int fd);

void moveBackward(int fd);

void steerLeft(int fd);

void steerRight(int fd);

void steerStraight(int fd);

void sendToArduino(int fd, int direction);
