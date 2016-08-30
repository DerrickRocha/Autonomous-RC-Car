/*
  This reads integers from a serial port.
 */
const int STOP = 0;
const int FORWARD = 1;
const int BACKWARD = 2;
const int LEFT = 3;
const int RIGHT = 4;
const int STEER_STRAIGHT = 5;
 
const int RIGHT_PIN  = 5;
const int LEFT_PIN  = 7;
const int BACKWARD_PIN  = 9;
const int FORWARD_PIN  = 12;
 
 
 
 

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  pinMode(FORWARD_PIN, OUTPUT);
  pinMode(BACKWARD_PIN, OUTPUT);
  pinMode(LEFT_PIN, OUTPUT);
  pinMode(RIGHT_PIN, OUTPUT);
}

// the loop routine runs over and over again forever:
void loop() {
  // read the input on serial port.
  int sensorValue = Serial.parseInt();
  // print out the value you read:
  switch(sensorValue){
  
    case STOP:
        
        digitalWrite(FORWARD_PIN,LOW);
        digitalWrite(BACKWARD_PIN,LOW);
        
        break;
        
    case FORWARD:
    
        digitalWrite(FORWARD_PIN,HIGH);
        digitalWrite(BACKWARD_PIN,LOW);
        
        break;

    case BACKWARD:

	 digitalWrite(FORWARD_PIN,LOW);
        digitalWrite(BACKWARD_PIN,HIGH);

	break;
        
    case STEER_STRAIGHT:
      
        digitalWrite(LEFT_PIN,LOW);
        digitalWrite(RIGHT_PIN,LOW);
        
        break;
      
   case LEFT:
        digitalWrite(LEFT_PIN,HIGH);
        digitalWrite(RIGHT_PIN,LOW);
        break;   
        
   case RIGHT:
        digitalWrite(LEFT_PIN,LOW);
        digitalWrite(RIGHT_PIN,HIGH);
        break;
  
  }
  
  
  Serial.println(sensorValue);
  //delay(1);        // delay in between reads for stability
}
