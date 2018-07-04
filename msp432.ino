//@Project: Pick and Place
//@Auther: Akash Dhamasia
//@DateCreated: 30-06-2018

#include <Wire.h>  
#include <Adafruit_PWMServoDriver.h>    

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();    

//#define MIN_PULSE_WIDTH 650  
//#define MAX_PULSE_WIDTH 2350  
//#define DEFAULT_PULSE_WIDTH 1500 
 
#define MIN_PULSE_WIDTH       544     // the shortest pulse sent to a servo  
#define MAX_PULSE_WIDTH      2400     // the longest pulse sent to a servo 
#define DEFAULT_PULSE_WIDTH  1500
#define FREQUENCY 40    

uint8_t base = 0;     
uint8_t shoulder = 3;    
uint8_t elbow = 7;    
uint8_t wrist_rot = 8;    
uint8_t wrist_ver = 10;    
uint8_t gripper = 12;   

int step_base = 0;
int step_shoulder = 45;
int step_elbow = 180;
int step_wrist_rot = 180;
int step_wrist_ver = 90;
int step_gripper = 10;

int base_angle = 0; 
int shoulder_angle = 90; //initialise angle = 90; go front on decreasing angle
int elbow_angle = 0; // go up with increasing angles
int wrist_rot_angle = 0; // 0 is facing downward
int wrist_ver_angle = 0;
int gripper_angle = 180;  //0 is open

void setup() {

  Serial.begin(9600);  
  pwm.begin();  
  pwm.setPWMFreq(FREQUENCY);  

  //Initialization functions and set up the initial position for Braccio
  //All the servo motors will be positioned in the "safety" position:
  //Base (M1):90 degrees
  //Shoulder (M2): 45 degrees
  //Elbow (M3): 180 degrees
  //Wrist vertical (M4): 180 degrees
  //Wrist rotation (M5): 90 degrees
  //gripper (M6): 10 degrees

  ServoMovement(30, 90, 45, 0, 180, 90,  10);  
  delay(2000);

  initialise();
  //ServoMovement(20, 90, 45, 45, 0, 90,  10);  
  //delay(1000);
}

int pulseWidth(int angle)  

{  
  int pulse_wide, analog_value;  
  pulse_wide = map(angle, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);  
  analog_value = int(float(pulse_wide) / 1000000 * FREQUENCY * 4096);  
  Serial.println(analog_value);  return analog_value;  
}    

int ServoMovement(int stepDelay, int vBase, int vShoulder, int vElbow,int vWrist_ver, int vWrist_rot, int vgripper) {

  // Check values, to avoid dangerous positions for the Braccio
      if (stepDelay > 30) stepDelay = 30;
  if (stepDelay < 10) stepDelay = 10;
  if (vBase < 0) vBase=0;
  if (vBase > 180) vBase=180;
  if (vShoulder < 15) vShoulder=15;
  if (vShoulder > 165) vShoulder=165;
  if (vElbow < 0) vElbow=0;
  if (vElbow > 180) vElbow=180;
  if (vWrist_ver < 0) vWrist_ver=0;
  if (vWrist_ver > 180) vWrist_ver=180;
  if (vWrist_rot > 180) vWrist_rot=180;
  if (vWrist_rot < 0) vWrist_rot=0;
      if (vgripper < 10) vgripper = 10;
  if (vgripper > 73) vgripper = vgripper;

  int exit = 1;

  //Until the all motors are in the desired position
  while (exit) 
  {     
    //For each servo motor if next degree is not the same of the previuos than do the movement    
    if (vBase != step_base) 
    {     
      //base.write(step_base);
      pwm.setPWM(base, 0, pulseWidth(step_base));  

      //One step ahead
      if (vBase > step_base) {
        step_base++;
      }
      //One step beyond
      if (vBase < step_base) {
        step_base--;
      }
    }

    if (vShoulder != step_shoulder)  
    {
      //shoulder.write(step_shoulder);
      pwm.setPWM(shoulder, 0, pulseWidth(step_shoulder));  

      //One step ahead
      if (vShoulder > step_shoulder) {
        step_shoulder++;
      }
      //One step beyond
      if (vShoulder < step_shoulder) {
        step_shoulder--;
      }

    }

    if (vElbow != step_elbow)  
    {
      //elbow.write(step_elbow);
      pwm.setPWM(elbow, 0, pulseWidth(step_elbow));  
       
      //One step ahead
      if (vElbow > step_elbow) {
        step_elbow++;
      }
      //One step beyond
      if (vElbow < step_elbow) {
        step_elbow--;
      }

    }

    if (vWrist_ver != step_wrist_rot) 
    {
      //wrist_rot.write(step_wrist_rot);
      pwm.setPWM(wrist_rot, 0, pulseWidth(step_wrist_rot));  

      //One step ahead
      if (vWrist_ver > step_wrist_rot) {
        step_wrist_rot++;       
      }
      //One step beyond
      if (vWrist_ver < step_wrist_rot) {
        step_wrist_rot--;
      }

    }

    if (vWrist_rot != step_wrist_ver)
    {
      //wrist_ver.write(step_wrist_ver);
      pwm.setPWM(wrist_ver, 0, pulseWidth(step_wrist_ver));  

      //One step ahead
      if (vWrist_rot > step_wrist_ver) {
        step_wrist_ver++;
      }
      //One step beyond
      if (vWrist_rot < step_wrist_ver) {
        step_wrist_ver--;
      }
    }

    if (vgripper != step_gripper)
    {
      //gripper.write(step_gripper);
      pwm.setPWM(gripper, 0, pulseWidth(step_gripper));  

      if (vgripper > step_gripper) {
        step_gripper++;
      }
      //One step beyond
      if (vgripper < step_gripper) {
        step_gripper--;
      }
    }
    
    //delay between each movement
    delay(stepDelay);
    
    //It checks if all the servo motors are in the desired position 
    if ((vBase == step_base) && (vShoulder == step_shoulder)
        && (vElbow == step_elbow) && (vWrist_ver == step_wrist_rot)
        && (vWrist_rot == step_wrist_ver) && (vgripper == step_gripper)) {
      step_base = vBase;
      step_shoulder = vShoulder;
      step_elbow = vElbow;
      step_wrist_rot = vWrist_ver;
      step_wrist_ver = vWrist_rot;
      step_gripper = vgripper;
      exit = 0;
    } else {
      exit = 1;
    }
  }
  
  return 0;   
}

int initialise()
{

  base_angle = 90; 
  shoulder_angle = 90; //initialise angle = 90; go front on decreasing angle
  elbow_angle = 0; // go up with increasing angles
  wrist_rot_angle = 100; // 0 is facing downward
  wrist_ver_angle = 0;
  gripper_angle = 10;

  //(step delay, M1, M2, M3, M4, M5, M6);
  //Braccio.ServoMovement(20, 90, 90, 0, 100, 90,  0);  

  ServoMovement(20, base_angle, shoulder_angle, elbow_angle, wrist_rot_angle, wrist_ver_angle,  gripper_angle);  

  //Wait 1 second
  delay(1000);
  return 0;

}

int forward()
{

  shoulder_angle = shoulder_angle - 1;
  elbow_angle = elbow_angle + 1;
  
  if(shoulder_angle <= 45){
    shoulder_angle = 45;
  }
  
  if(elbow_angle >= 45 ){
    elbow_angle = 45;
  }
            
  ServoMovement(20, base_angle, shoulder_angle, elbow_angle, wrist_rot_angle, wrist_ver_angle,  gripper_angle);  
  return 0;
}

int backward()
{

  shoulder_angle = shoulder_angle + 1;
  elbow_angle = elbow_angle - 1;
  
  if(shoulder_angle >= 90){
    shoulder_angle = 90;
  }

  if(elbow_angle <= 0 ){
    elbow_angle = 0;
  }
   
  ServoMovement(20, base_angle, shoulder_angle, elbow_angle, wrist_rot_angle, wrist_ver_angle,  gripper_angle);  
  return 0;
}

int rotate_left()
{

  base_angle = base_angle - 1;
  
  if(base_angle <= 0){
    base_angle = 0;
  }
              
  ServoMovement(20, base_angle, shoulder_angle, elbow_angle, wrist_rot_angle, wrist_ver_angle,  gripper_angle);  
  return 0;
}

int rotate_right()
{

  base_angle = base_angle + 1;
  
  if(base_angle >= 180){
    base_angle = 180;
  }
              
  ServoMovement(20, base_angle, shoulder_angle, elbow_angle, wrist_rot_angle, wrist_ver_angle,  gripper_angle);  
  return 0;
}


int pickup()
{

  int temp_elbow_angle = 45;
  base_angle = base_angle + 7;

  ServoMovement(20, base_angle, shoulder_angle, temp_elbow_angle, wrist_rot_angle, 0,  gripper_angle);  
  delay(1000);

  wrist_rot_angle = 0;
  ServoMovement(20, base_angle, shoulder_angle, temp_elbow_angle, wrist_rot_angle, 0,  gripper_angle);  
  delay(1000);

  ServoMovement(20, base_angle, shoulder_angle, temp_elbow_angle, wrist_rot_angle, wrist_ver_angle,  gripper_angle);  
  delay(1000);

  temp_elbow_angle = elbow_angle;
  ServoMovement(20, base_angle, shoulder_angle, temp_elbow_angle, wrist_rot_angle, wrist_ver_angle,  gripper_angle);  
  delay(1000);

  int temp_shoulder_angle = 45;

  if(elbow_angle < 10){
  temp_shoulder_angle = 45;
  }
  
  ServoMovement(20, base_angle, temp_shoulder_angle, temp_elbow_angle, wrist_rot_angle, wrist_ver_angle,  gripper_angle);  
  delay(1000);

  //Braccio.ServoMovement(20, 90,  35, elbow_angle, 0, 90,  10);  
  //Wait 1 second
  //delay(1000);

  gripper_angle = 80;
  ServoMovement(20, base_angle, temp_shoulder_angle, temp_elbow_angle, wrist_rot_angle, wrist_ver_angle,  gripper_angle);  
  delay(1000);

  wrist_rot_angle = 90;
  temp_elbow_angle = 45;
  elbow_angle = temp_elbow_angle;
  ServoMovement(20, base_angle, shoulder_angle, temp_elbow_angle, wrist_rot_angle, wrist_ver_angle,  gripper_angle);  
  delay(1000);

  droppackage();
  return 0;  
}

int droppackage()
{

  base_angle = 0;
  ServoMovement(20, base_angle, shoulder_angle, elbow_angle, wrist_rot_angle, wrist_ver_angle,  gripper_angle);  
  delay(1000);

  gripper_angle = 10;
  ServoMovement(20, base_angle, shoulder_angle, elbow_angle, wrist_rot_angle, wrist_ver_angle,  gripper_angle);  
  delay(1000);

  initialise();
  return 0;
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

  //Braccio.ServoMovement(20, 90, 45, 45, 0, 90,  0);  

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
           wrist_ver_angle = 180 - wrist_ver_angle; 
        }
        
        pickup();
        delay(10);      
      }

  }
  
}

 
