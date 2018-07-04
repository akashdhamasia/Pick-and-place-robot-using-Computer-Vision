//@Project: Pick and Place
//@Auther: Akash Dhamasia
//@DateCreated: 30-06-2018

/*
  simpleMovements.ino

 This  sketch simpleMovements shows how they move each servo motor of Braccio

 Created on 18 Nov 2015
 by Andrea Martino

 This example is in the public domain.
 */

#include <Braccio.h>
#include <Servo.h>

Servo base;
Servo shoulder;
Servo elbow;
Servo wrist_rot;
Servo wrist_ver;
Servo gripper;


int base_angle = 0; 
int shoulder_angle = 90; //initialise angle = 90; go front on decreasing angle
int elbow_angle = 0; // go up with increasing angles
int wrist_rot_angle = 0; // 0 is facing downward
int wrist_ver_angle = 0;
int gripper_angle = 0;  //0 is open

void setup() {

  Serial.begin(9600);
  //Initialization functions and set up the initial position for Braccio
  //All the servo motors will be positioned in the "safety" position:
  //Base (M1):90 degrees
  //Shoulder (M2): 45 degrees
  //Elbow (M3): 180 degrees
  //Wrist vertical (M4): 180 degrees
  //Wrist rotation (M5): 90 degrees
  //gripper (M6): 10 degrees
  Braccio.begin();

  Braccio.ServoMovement(20, 90, 45, 0, 180, 0,  10);  
  delay(2000);

  initialise();
}

void loop() {
   /*
   Step Delay: a milliseconds delay between the movement of each servo.  Allowed values from 10 to 30 msec.
   M1=base degrees. Allowed values from 0 to 180 degrees
   M2=shoulder degrees. Allowed values from 15 to 165 degrees
   M3=elbow degrees. Allowed values from 0 to 180 degrees
   M4=wrist vertical degrees. Allowed values from 0 to 180 degrees
   M5=wrist rotation degrees. Allowed values from 0 to 180 degrees
   M6=gripper degrees. Allowed values from 10 to 73 degrees. 10: the toungue is open, 73: the gripper is closed.
  */

//  Braccio.ServoMovement(20, 90, 45, 45, 0, 90,  0);  

  if (Serial.available()) {
    
      char serialListener = Serial.read();
      if (serialListener == 'i') {
        
        initialise();  
      }
      else if (serialListener == 'f') {

        forward();
        delay(10);      
      }
      else if (serialListener == 'b') {

        backward();
        delay(10);      
      }
      else if (serialListener == 'v') {
        
        rotate_left();
        delay(10);      
      }
      else if (serialListener == 'r') {
        
        rotate_right();
        delay(10);      
      }
      else if (serialListener == 'p') {

        delay(1000);      
 
        if (Serial.available()){
           wrist_ver_angle = Serial.parseInt();
        }
        
        pickup();
        delay(10);      
      }

  }
  
}

void initialise()
{

  base_angle = 90; 
  shoulder_angle = 90; //initialise angle = 90; go front on decreasing angle
  elbow_angle = 0; // go up with increasing angles
  wrist_rot_angle = 100; // 0 is facing downward
  wrist_ver_angle = 0;
  gripper_angle = 10;

  //(step delay, M1, M2, M3, M4, M5, M6);
  //Braccio.ServoMovement(20, 90, 90, 0, 100, 90,  0);  

  Braccio.ServoMovement(20, base_angle, shoulder_angle, elbow_angle, wrist_rot_angle, wrist_ver_angle,  gripper_angle);  

  //Wait 1 second
  delay(1000);

}

void forward()
{

  shoulder_angle = shoulder_angle - 1;
  elbow_angle = elbow_angle + 1;
  
  if(shoulder_angle <= 45){
    shoulder_angle = 45;
  }
  
  if(elbow_angle >= 45 ){
    elbow_angle = 45;
  }
            
  Braccio.ServoMovement(20, base_angle, shoulder_angle, elbow_angle, wrist_rot_angle, wrist_ver_angle,  gripper_angle);  

}

void backward()
{

  shoulder_angle = shoulder_angle + 1;
  elbow_angle = elbow_angle - 1;
  
  if(shoulder_angle >= 90){
    shoulder_angle = 90;
  }

  if(elbow_angle <= 0 ){
    elbow_angle = 0;
  }
   
  Braccio.ServoMovement(20, base_angle, shoulder_angle, elbow_angle, wrist_rot_angle, wrist_ver_angle,  gripper_angle);  
  
}

void rotate_left()
{

  base_angle = base_angle - 1;
  
  if(base_angle <= 0){
    base_angle = 0;
  }
              
  Braccio.ServoMovement(20, base_angle, shoulder_angle, elbow_angle, wrist_rot_angle, wrist_ver_angle,  gripper_angle);  

}

void rotate_right()
{

  base_angle = base_angle + 1;
  
  if(base_angle >= 180){
    base_angle = 180;
  }
              
  Braccio.ServoMovement(20, base_angle, shoulder_angle, elbow_angle, wrist_rot_angle, wrist_ver_angle,  gripper_angle);  

}


void pickup()
{

  int temp_elbow_angle = 90;

  Braccio.ServoMovement(20, base_angle, shoulder_angle, temp_elbow_angle, wrist_rot_angle, 0,  gripper_angle);  
  delay(1000);

  wrist_rot_angle = 0;
  Braccio.ServoMovement(20, base_angle, shoulder_angle, temp_elbow_angle, wrist_rot_angle, 0,  gripper_angle);  
  delay(1000);

  Braccio.ServoMovement(20, base_angle, shoulder_angle, temp_elbow_angle, wrist_rot_angle, wrist_ver_angle,  gripper_angle);  
  delay(1000);

  temp_elbow_angle = elbow_angle;
  Braccio.ServoMovement(20, base_angle, shoulder_angle, temp_elbow_angle, wrist_rot_angle, wrist_ver_angle,  gripper_angle);  
  delay(1000);

  Braccio.ServoMovement(20, base_angle, 40, temp_elbow_angle, wrist_rot_angle, wrist_ver_angle,  gripper_angle);  
  delay(1000);

  //Braccio.ServoMovement(20, 90,  35, elbow_angle, 0, 90,  10);  
  //Wait 1 second
  //delay(1000);

  gripper_angle = 73;
  Braccio.ServoMovement(20, base_angle, shoulder_angle, temp_elbow_angle, wrist_rot_angle, wrist_ver_angle,  gripper_angle);  
  delay(1000);

  wrist_rot_angle = 90;
  temp_elbow_angle = 45;
  elbow_angle = temp_elbow_angle;
  Braccio.ServoMovement(20, base_angle, shoulder_angle, temp_elbow_angle, wrist_rot_angle, wrist_ver_angle,  gripper_angle);  
  delay(1000);

  droppackage();
  
}

void droppackage()
{

  base_angle = 180;
  Braccio.ServoMovement(20, base_angle, shoulder_angle, elbow_angle, wrist_rot_angle, wrist_ver_angle,  gripper_angle);  
  delay(1000);

  gripper_angle = 10;
  Braccio.ServoMovement(20, base_angle, shoulder_angle, elbow_angle, wrist_rot_angle, wrist_ver_angle,  gripper_angle);  
  delay(1000);

  initialise();
}


 
