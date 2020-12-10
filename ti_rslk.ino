#include "ECE3.h"
////////////////////////////////////
///////////Defining Pins////////////
////////////////////////////////////

const int left_nslp_pin = 31;
const int left_dir_pin = 29;
const int left_pwm_pin = 40;

const int right_nslp_pin = 11;
const int right_dir_pin = 30;
const int right_pwm_pin = 39;

////////////////////////////////////
//////////////Globals///////////////
////////////////////////////////////

//IR sensor array
uint16_t sensorValues[8];

//For calibrating the sensors
int rawCombined;
int prevRawCombined;
int fused;
int prevFused;
int error;

//Default speed if there is no error
int leftDefaultSpeed = 150;
int rightDefaultSpeed = 155;

//Keeps track of what the car is doing
int counter;

void setup() {
    //Serial.begin(9600);
    ECE3_Init();

    //Defining pins
    pinMode(31,OUTPUT);
    pinMode(29,OUTPUT);
    pinMode(40,OUTPUT);
    pinMode(11,OUTPUT);
    pinMode(30,OUTPUT);
    pinMode(39,OUTPUT);

    //Car is on and travelling forward
    digitalWrite(29,LOW);
    digitalWrite(31,HIGH);
    digitalWrite(30,LOW);
    digitalWrite(11,HIGH);
    
    //Car starts at "followPath()"
    counter = 0;

    //Initial previous fused value
    prevFused = 0;

    delay(5000);
}

void loop() {  
    
    sensorFusion();
    
    //counter = 0: away from start, counter = 2: return to start
    if(counter == 0 || counter == 2)
        followPath();
    
    //counter = 1: do a doughnut
    if(counter == 1)
        doughnut();
   
    //counter = 3: finish
    if(counter == 3)
        finish();
}

////////////////////////////////////
///////////Sensor Fusion////////////
////////////////////////////////////

void sensorFusion() {
    
  //Read raw sensor values
  ECE3_read_IR(sensorValues);

  //Normalize all sensors to be between (0-1000)
  int normal0 = ((sensorValues[0]-689)*1000)/1811;
  int normal1 = ((sensorValues[1]-574)*1000)/1926;
  int normal2 = ((sensorValues[2]-597)*1000)/1903;
  int normal3 = ((sensorValues[3]-597)*1000)/1902;
  int normal4 = ((sensorValues[4]-666)*1000)/1833;
  int normal5 = ((sensorValues[5]-643)*1000)/1857;
  int normal6 = ((sensorValues[6]-574)*1000)/1925;
  int normal7 = ((sensorValues[7]-666)*1000)/1834;

  //Produce one fused value for all sensors
  fused = ((-8*normal0)-(4*normal1)-(2*normal2)-(normal3)+(normal4)+(2*normal5)+(4*normal6)+(8*normal7));  
}

////////////////////////////////////
///////////Follow Path//////////////
////////////////////////////////////

void followPath() {  
    
    int leftSpeed;
    int rightSpeed;
    int PDcontroller;
    int steer;

    //Arbitrary constants tuned for this system; Kp = 2 & Kd = 36 seem to work well.
    const int Kp = 2;
    const int Kd = 36;

    //Error is current fused value minus previous fused value
    error = fused - prevFused;

    //PD Controller
    PDcontroller = (Kp*fused)+(Kd*error);    
    PDcontroller = constrain(PDcontroller,-10000,10000);

    //Translate PD control value into readable value to steer the car
    steer = map(PDcontroller,-10000,10000,-100,100);

    //For debugging purposes
    //Serial.println(speed);
 
    //Define left and right speed based on default speed and steer value
    leftSpeed = (leftDefaultSpeed - steer);
    rightSpeed = (rightDefaultSpeed + steer);

    //Drive
    analogWrite(40,leftSpeed);
    analogWrite(39,rightSpeed);

    //If the car hits a black line, engage "doughnut" or "finish"
    if(prevFused == 0 && steer == 0)
        counter++;
    
    //Set current fused value as "prev" for next iteration
    prevFused = fused;
}

////////////////////////////////////
//////////////Doughnut//////////////
////////////////////////////////////

void doughnut() {  
    
    //Change direction of left wheel
    digitalWrite(29,HIGH);

    //Donut for 200 ms
    analogWrite(40,255);
    analogWrite(39,255);
    delay(200);

    //Change direction of left wheel and drive forward for 75 ms as a transitional buffer
    digitalWrite(29,LOW);
    analogWrite(40,leftDefaultSpeed);
    analogWrite(39,rightDefaultSpeed);
    delay(75);

    //Return to "followPath"
    counter++;
}

////////////////////////////////////
////////////////Finish//////////////
////////////////////////////////////

void finish() {  
 
    //Turn direction of left wheel
    digitalWrite(29,HIGH);

    //Donut for 100 ms
    analogWrite(40,255);
    analogWrite(39,255);
    delay(100);

    //Stop car
    digitalWrite(31,LOW);
    digitalWrite(11,LOW);
}
