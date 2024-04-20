 #include<AFMotor.h>
 #include <Servo.h>
#include <NewPing.h>
const int buttonPin =  9;     // the number of the pushbutton pin
#define RIGHT A5              // Right IR sensor connected to analog pin A5 of Arduino Uno:
#define LEFT A4              // Left IR sensor connected to analog pin A4 of Arduino Uno:
// #define TRIGGER_PIN A2        // Trigger pin connected to analog pin A2 of Arduino Uno:
#define TRIGGER_PIN A0 
  
#define ECHO_PIN A1         // Echo pin connected to analog pin A3 of Arduino Uno:
#define MAX_SPEED 200 // sets speed of DC traction motors to 150/250 or about 70% of full speed - to get power drain down.
#define MAX_SPEED_OFFSET 40 // this sets offset to allow for differences between the two DC traction motors
#define COLL_DIST 35 // sets distance at which robot stops and reverses to 35cm
#define TURN_DIST COLL_DIST+20 // sets distance at which robot veers away from object
AF_DCMotor leftMotor1(1, MOTOR12_1KHZ); // create motor #1 using M1 output on Motor Drive Shield, set to 1kHz PWM frequency
AF_DCMotor leftMotor2(4, MOTOR12_1KHZ); // create motor #2, using M2 output, set to 1kHz PWM frequency
AF_DCMotor rightMotor1(2, MOTOR34_1KHZ);// create motor #3, using M3 output, set to 1kHz PWM frequency
AF_DCMotor rightMotor2(3, MOTOR34_1KHZ);// create motor #4, using M4 output, set to 1kHz PWM frequency
unsigned int distanceOF = 0;    //Variable to store ultrasonic sensor distance:
unsigned int Right_Value = 0; //Variable to store Right IR sensor value:
unsigned int Left_Value = 0;  //Variable to store Left IR sensor value:
int leftDistance, rightDistance; //distances on either side
int curDist = 0;
String motorSet = "";
int speedSet = 0;
Servo myservo;  // create servo object to control a servo 
  


// variables will change:
int  initial    = 0;       //hold current  initial
int oldstate    = 0;       //hold last  initial
int buttonstate = 0;      // variable for reading the pushbutton status
char direct;
int speedBlue=200;

void setup() {
  pinMode(buttonPin, INPUT); // initialize the pushbutton pin as an input:
  pinMode(RIGHT, INPUT); //set analog pin RIGHT as an input:
   pinMode(LEFT, INPUT);  //set analog pin RIGHT as an input:
   myservo.attach(10);  // attaches the servo on pin 10 (SERVO_1 on the Motor Drive Shield to the servo object 
  myservo.write(90); // tells the servo to position at 90-degrees ie. facing forward.
  delay(1000); // delay for one seconds
  Serial.begin(9600);
}
void loop(){
  //debouncing routline to read button
  buttonstate = digitalRead(buttonPin);  //state the  initial of button
  if(buttonstate == HIGH){               //check if it has been pressed 
    delay(50);
    buttonstate = digitalRead(buttonPin);//state button again
    if(buttonstate == LOW){              //if it is 0 considered one press
     initial = ++oldstate;       //increase  initial by 1
    }
  }
  else{                          //check if it has been NOT pressed
      delay(100);
      }
      Serial.print("Initial is=");
      Serial.println(initial);
      oldstate=initial;
   switch (initial){               //react to button press a  initial
  //    case 1:  
  //    Serial.println("Initial=1");                                                                                               //obstacle avoidance
  //      myservo.write(90);  // move eyes forward
  // delay(90);
  // curDist = readPing();   // read distance
  // Serial.println(curDist);
  // if (curDist < COLL_DIST) {changePath();}  // if forward is blocked change direction
  // moveForward();  // move forward
  // delay(500);
  // oldstate=initial;
  //      break;
     case 2:  
     Serial.println("Initial=2");                                                                                                  // object following
       delay(50);                                        //wait 50ms between pings:
distanceOF = readPing();                       //send ping, get distance in cm and store it in 'distance' variable:
Serial.print("distance");                   
Serial.println(distanceOF);                         // print the distance in serial monitor:


    Right_Value = digitalRead(RIGHT);             // read the value from Right IR sensor:
    Left_Value = digitalRead(LEFT);               // read the value from Left IR sensor:
 
Serial.print("RIGHT");                      
Serial.println(Right_Value);                      // print the right IR sensor value in serial monitor:
Serial.print("LEFT");                       
Serial.println(Left_Value);                       //print the left IR sensor value in serial monitor:

if((distanceOF > 1) && (distanceOF < 15)){            //check wheather the ultrasonic sensor's value stays between 1 to 15.
                                                  //If the condition is 'true' then the statement below will execute:
  //Move Forward:
  rightMotor1.setSpeed(200);  //define rightrightMotor1 speed:
  rightMotor1.run(FORWARD);   //rotate rightMotor1 clockwise:
  rightMotor2.setSpeed(200);  //define rightMotor2 speed:
  rightMotor2.run(FORWARD);   //rotate rightMotor2 clockwise:
  leftMotor1.setSpeed(200);  //define leftMotor1 speed:
  leftMotor1.run(FORWARD);   //rotate leftMotor1 clockwise:
  leftMotor2.setSpeed(200);  //define leftMotor2 speed:
  leftMotor2.run(FORWARD);   //rotate leftMotor2 clockwise:
  
}else if((Right_Value==0) && (Left_Value==1)) {   //If the condition is 'true' then the statement below will execute:
  
  //Turn Left                                                
  rightMotor1.setSpeed(200);  //define rightMotor1 speed:
  rightMotor1.run(FORWARD);   //rotate rightMotor1 cloclwise:
  rightMotor2.setSpeed(200);  //define rightMotor2 speed:
  rightMotor2.run(FORWARD);   //rotate rightMotor2 clockwise:
  leftMotor1.setSpeed(250);  //define leftMotor1 speed:
  leftMotor1.run(BACKWARD);  //rotate leftMotor1 anticlockwise:
  leftMotor2.setSpeed(250);  //define leftMotor2 speed:
  leftMotor2.run(BACKWARD);  //rotate leftMotor2 anticlockwise:
  delay(150);
  
}else if((Right_Value==1)&&(Left_Value==0)) {     //If the condition is 'true' then the statement below will execute:
  
  //Turn Right
  rightMotor1.setSpeed(250);  //define rightMotor1 speed:
  rightMotor1.run(BACKWARD);  //rotate rightMotor1 anticlockwise:
  rightMotor2.setSpeed(250);  //define rightMotor2 speed:
  rightMotor2.run(BACKWARD);  //rotate rightMotor2 anticlockwise:
  leftMotor1.setSpeed(200);  //define leftMotor1 speed:
  leftMotor1.run(FORWARD);   //rotate leftMotor1 clockwise:
  leftMotor2.setSpeed(200);  //define leftMotor2 speed:
  leftMotor2.run(FORWARD);   //rotate leftMotor2 clockwise:
  delay(150);
  
}else if(distanceOF > 15) {                          //If the condition is 'true' then the statement below will execute:
  
  //Stop
  rightMotor1.setSpeed(0);    //define rightMotor1 speed:
  rightMotor1.run(RELEASE);   //stop rightMotor1:
  rightMotor2.setSpeed(0);    //define rightMotor2 speed:
  rightMotor2.run(RELEASE);   //stop rightMotor2:
  leftMotor1.setSpeed(0);    //define leftMotor1 speed:
  leftMotor1.run(RELEASE);   //stop leftMotor1:
  leftMotor2.setSpeed(0);    //define leftMotor2 speed:
  leftMotor2.run(RELEASE);   //stop leftMotor2:
}
       oldstate =  initial;
       break;        
    case 1: 
    Serial.println("Initial=3");                                                                                 //bluetooth car
    if(Serial.available()>0)
 {
   direct=Serial.read();
   if(direct=='U')
   moveForwardBlue();
   else if(direct=='D')
   moveBackwardBlue();
   else if(direct=='L')
   moveLeftBlue();
   else if(direct=='R')
   moveRightBlue();
   else if(direct=='S')
   stopBlue();
 }
oldstate=initial;

    break;
            
     }
}
int readPing() { // read the ultrasonic sensor distance
  delay(70);  
  long duration;
  long distance;
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(ECHO_PIN, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2;
  // Prints the distance on the Serial Monitor
  Serial.print("Distance: ");
  Serial.println(distance);
  return distance;
}
void changePath() {
  moveStop();   // stop forward movement
  myservo.write(30);  // check distance to the right
    delay(500);
    rightDistance = readPing(); //set right distance
    Serial.println(rightDistance);
    delay(500);
    myservo.write(150);  // check distace to the left
    delay(700);
    leftDistance = readPing(); //set left distance
    delay(500);
    myservo.write(90); //return to center
    delay(100);
    compareDistance();
  }
  void compareDistance()   // find the longest distance
{
  if (leftDistance>rightDistance) //if left is less obstructed 
  {
    turnLeft();
  }
  else if (rightDistance>leftDistance) //if right is less obstructed
  {
    turnRight();
  }
   else //if they are equally obstructed
  {
    turnAround();
  }
}
void moveStop() {leftMotor1.run(RELEASE); leftMotor2.run(RELEASE); rightMotor1.run(RELEASE); rightMotor2.run(RELEASE);}  // stop the motors.
void moveForward() {
    motorSet = "FORWARD";
    leftMotor1.run(FORWARD);      // turn it on going forward
    leftMotor2.run(FORWARD);      // turn it on going forward
    rightMotor1.run(FORWARD);     // turn it on going forward
    rightMotor2.run(FORWARD);     // turn it on going forward
  for (speedSet = 0; speedSet < MAX_SPEED; speedSet +=2) // slowly bring the speed up to avoid loading down the batteries too quickly
  {
    leftMotor1.setSpeed(speedSet);
    leftMotor2.setSpeed(speedSet);
    rightMotor1.setSpeed(speedSet); 
    rightMotor2.setSpeed(speedSet);
    delay(5);
  }
}
//-------------------------------------------------------------------------------------------------------------------------------------
void moveBackward() {
    motorSet = "BACKWARD";
    leftMotor1.run(BACKWARD);     // turn it on going backward
    leftMotor2.run(BACKWARD);     // turn it on going backward
    rightMotor1.run(BACKWARD);    // turn it on going backward
    rightMotor2.run(BACKWARD);    // turn it on going backward
  for (speedSet = 0; speedSet < MAX_SPEED; speedSet +=2) // slowly bring the speed up to avoid loading down the batteries too quickly
  {
    leftMotor1.setSpeed(speedSet);
    leftMotor2.setSpeed(speedSet);
    rightMotor1.setSpeed(speedSet); 
    rightMotor2.setSpeed(speedSet); 
    delay(5);
  }
}  
//-------------------------------------------------------------------------------------------------------------------------------------
void turnRight() {
  motorSet = "RIGHT";
  leftMotor1.run(FORWARD);      // turn motor 1 forward
  leftMotor2.run(FORWARD);      // turn motor 2 forward
  rightMotor1.run(BACKWARD);    // turn motor 3 backward
  rightMotor2.run(BACKWARD);    // turn motor 4 backward
  rightMotor1.setSpeed(speedSet+MAX_SPEED_OFFSET);      
  rightMotor2.setSpeed(speedSet+MAX_SPEED_OFFSET);     
  delay(1500); // run motors this way for 1500        
  motorSet = "FORWARD";
  leftMotor1.run(FORWARD);      // set both motors back to forward
  leftMotor2.run(FORWARD);
  rightMotor1.run(FORWARD);
  rightMotor2.run(FORWARD);      
}  
//-------------------------------------------------------------------------------------------------------------------------------------
void turnLeft() {
  motorSet = "LEFT";
  leftMotor1.run(BACKWARD);      // turn motor 1 backward
  leftMotor2.run(BACKWARD);      // turn motor 2 backward
  leftMotor1.setSpeed(speedSet+MAX_SPEED_OFFSET);     
  leftMotor2.setSpeed(speedSet+MAX_SPEED_OFFSET);    
  rightMotor1.run(FORWARD);     // turn motor 3 forward
  rightMotor2.run(FORWARD);     // turn motor 4 forward
  delay(1500); // run motors this way for 1500  
  motorSet = "FORWARD";
  leftMotor1.run(FORWARD);      // turn it on going forward
  leftMotor2.run(FORWARD);      // turn it on going forward
  rightMotor1.run(FORWARD);     // turn it on going forward
  rightMotor2.run(FORWARD);     // turn it on going forward
}  
//-------------------------------------------------------------------------------------------------------------------------------------
void turnAround() {
  motorSet = "RIGHT";
  leftMotor1.run(FORWARD);      // turn motor 1 forward
  leftMotor2.run(FORWARD);      // turn motor 2 forward
  rightMotor1.run(BACKWARD);    // turn motor 3 backward
  rightMotor2.run(BACKWARD);    // turn motor 4 backward
  rightMotor1.setSpeed(speedSet+MAX_SPEED_OFFSET);      
  rightMotor2.setSpeed(speedSet+MAX_SPEED_OFFSET);
  delay(1700); // run motors this way for 1700
  motorSet = "FORWARD";
  leftMotor1.run(FORWARD);      // set both motors back to forward
  leftMotor2.run(FORWARD);
  rightMotor1.run(FORWARD);
  rightMotor2.run(FORWARD);      
}  
//=========================================================================================================================================
void moveForwardBlue() {
    leftMotor1.run(FORWARD);
    leftMotor1.setSpeed(speedBlue);     
    leftMotor2.run(FORWARD);      
    leftMotor2.setSpeed(speedBlue);
    rightMotor1.run(FORWARD);    
    rightMotor1.setSpeed(speedBlue);
    rightMotor2.run(FORWARD);     
    rightMotor2.setSpeed(speedBlue);
}
void moveBackwardBlue() {
    leftMotor1.run(BACKWARD);
    leftMotor1.setSpeed(speedBlue);     
    leftMotor2.run(BACKWARD);      
    leftMotor2.setSpeed(speedBlue);
    rightMotor1.run(BACKWARD);    
    rightMotor1.setSpeed(speedBlue);
    rightMotor2.run(BACKWARD);     
    rightMotor2.setSpeed(speedBlue);
}
void moveLeftBlue() {
    leftMotor1.run(BACKWARD);
    leftMotor1.setSpeed(speedBlue);     
    leftMotor2.run(BACKWARD);      
    leftMotor2.setSpeed(speedBlue);
    rightMotor1.run(FORWARD);    
    rightMotor1.setSpeed(speedBlue);
    rightMotor2.run(FORWARD);     
    rightMotor2.setSpeed(speedBlue);
}
void moveRightBlue() {
    leftMotor1.run(FORWARD);
    leftMotor1.setSpeed(speedBlue);     
    leftMotor2.run(FORWARD);      
    leftMotor2.setSpeed(speedBlue);
    rightMotor1.run(BACKWARD);    
    rightMotor1.setSpeed(speedBlue);
    rightMotor2.run(BACKWARD);     
    rightMotor2.setSpeed(speedBlue);
}
void stopBlue() {
    leftMotor1.run(RELEASE);    
    leftMotor2.run(RELEASE);      
    rightMotor1.run(RELEASE);    
    rightMotor2.run(RELEASE);     
}