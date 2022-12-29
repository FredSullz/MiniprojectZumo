#include <Wire.h>
#include <Zumo32U4.h>

Zumo32U4IMU imu;
Zumo32U4LCD display;
Zumo32U4Motors motors;
Zumo32U4Buzzer buzzer;
Zumo32U4ButtonA buttonA;
Zumo32U4Encoders encoders;
Zumo32U4LineSensors lineSensors;
Zumo32U4ProximitySensors proxSensors;


//Setup the linesensor array and the variabels it works with.
int lineSensorValues[5]; 
bool useEmitters = true;
int16_t error = 0;
int leftValue=0;
int mLeftValue=0; 
int midValue=0;
int mRightValue=0;
int rightValue=0;
int linesPlayed=0;

//The imu variabels.
float gyroOffsetZ = 0;
float angleG = 0;
float angleM = 0;
float angle  = 0;

//Encoder variabels
int16_t headCountsLeft = 0;
int16_t headCountsRight = 0;

//Proximity sensor variabels
int levelCount = 6;
int brightLevels[6] = {0.4,0.8,2,3,7,8};
int brightness[2];

// The speed variabels.
int leftSpeed = 0;
int rightSpeed = 0;
int heading =0;

//Tells which mode we are in
int mode = 1; 

void calibrateSensors()
{
  // Wait 1 second and then begin automatic sensor calibration
  // by rotating in place to sweep the sensors over the line
  delay(1000);
  for(uint16_t i = 0; i < 120; i++)
  {
    if (i > 30 && i <= 90)
    {
      motors.setSpeeds(-100, 100);
    }
    else
    {
      motors.setSpeeds(100, -100);
    }
    lineSensors.calibrate();
  }
  motors.setSpeeds(0, 0);
}

void calibrateGyro(){
  display.print(F("Gryo Cal"));
  delay(500);

  //Korrigere for gryo offset 
  for (uint16_t i = 0; i < 1024; i++){
  // Wait for new data to be available, then read it.
    while(!imu.gyroDataReady()){}
    imu.readGyro();

    // Add the Y axis reading to the total.
    gyroOffsetZ += imu.g.z;
  }
  gyroOffsetZ /= 1024;
}

void setup() {
  Serial.begin(9600);

  //Initiate and set proxysensor
  proxSensors.initFrontSensor();
  proxSensors.setBrightnessLevels(brightLevels, levelCount);
  
  //Initiate and calibate the IR sensors 
  display.clear();
  display.print(F("Press A"));
  display.gotoXY(0, 1);
  display.print(F("to cal L"));
  buttonA.waitForButton();
  display.clear();
   
  lineSensors.initFiveSensors(); 
  lineSensors.emittersOn();  
  calibrateSensors(); 

  //Initiate and calibate the gyro and magnetometer
  display.clear();
  display.print(F("Press A"));
  display.gotoXY(0, 1);
  display.print(F("to cal G"));
  buttonA.waitForButton();
  display.clear();
  
  Wire.begin(); 
  imu.init();
  imu.enableDefault();
  calibrateGyro();

  //Wating to start
  display.clear();
  display.print(F("Press A"));
  display.gotoXY(0, 1);
  display.print(F("to start"));
  buttonA.waitForButton();
  display.clear();
  delay(1000);
}

void Angle(){
  static uint16_t lastUpdate = 0;
  uint16_t m = micros();
  uint16_t dt = m - lastUpdate;
  lastUpdate = m;
  
  imu.readGyro();
  imu.readMag();
 
  angleG =((float)imu.g.z-gyroOffsetZ)*dt*9/1000000000;

  angleM =((float)imu.m.z);

  angle += (angleG);
}

void finder(int leftValue,int mLeftValue,int midValue,int mRightValue,int rightValue) {
  //Mode 1
  int16_t position = lineSensors.readLine(lineSensorValues,true);
  lineSensors.emittersOn();

  error = - angle;
  leftSpeed = 400 - error*14;
  rightSpeed = 400 + error*14;
  motors.setSpeeds(leftSpeed, rightSpeed);
  
  if(leftValue<300|mLeftValue<300|midValue<300|mRightValue<300|rightValue<300){
    motors.setSpeeds(150, 150);

    delay(230);

    motors.setSpeeds(150, -150);
    while(true){
      int16_t position = lineSensors.readLine(lineSensorValues,true);
      lineSensors.emittersOn();

      lineSensors.read(lineSensorValues, useEmitters ? QTR_EMITTERS_ON : QTR_EMITTERS_OFF);
      leftValue = lineSensorValues[0];
      mLeftValue = lineSensorValues[1];
      midValue = lineSensorValues[2];
      mRightValue = lineSensorValues[3];
      rightValue = lineSensorValues[4];
      lineSensors.emittersOn(); 

      if((2030>position && position>1970) && (leftValue<300|mLeftValue<300|midValue<300|mRightValue<300|rightValue<300)){
        mode=2;
        break;
      }
    }
  }
}

void lineFollow(int leftValue,int mLeftValue,int midValue,int mRightValue,int rightValue){
  //Mode 2
  int16_t position = lineSensors.readLine(lineSensorValues, true);
  lineSensors.emittersOn();

  error = position - 2000;
  
  leftSpeed = 150 -error/4;
  rightSpeed = 150 + error/4;

  motors.setSpeeds(leftSpeed, rightSpeed);

  if(leftValue<500 | rightValue<500){
    //STOP
    delay(5);
    motors.setSpeeds(0, 0);
    mode=3;
    delay(1000);
    lineSensors.emittersOn();
  }
}

void detect(){
  //Mode 3
  
  proxSensors.read();
  brightness[0] = proxSensors.countsFrontWithLeftLeds();
  brightness[1] = proxSensors.countsFrontWithRightLeds(); 

  display.gotoXY(0, 0);
  display.print(brightness[0]);

  display.gotoXY(0, 1);
  display.print(brightness[1]);
  
  if(brightness[0]==4 && brightness[1]==4){
    heading=angle;
    headCountsLeft = encoders.getCountsLeft();
    headCountsRight = encoders.getCountsRight();   
    mode=4;
  }
  else if(brightness[0]==6 && brightness[1]==6){
    heading=angle;
    mode=6;
  }
}

void trash1(int leftValue,int mLeftValue,int midValue,int mRightValue,int rightValue){
  //Mode 4 
  error = heading - angle;
  leftSpeed =  350 - error*14;
  rightSpeed = 350 + error*14;
  motors.setSpeeds(leftSpeed, rightSpeed);

  if(leftValue<300|mLeftValue<300|midValue<300|mRightValue<300|rightValue<300&& headCountsLeft<encoders.getCountsLeft()-700){
    mode=5;
  }
}

void reverse1(){
  //mode 5
  error = heading - 1 - angle;
  leftSpeed =  350 - error*14;
  rightSpeed = 350 + error*14;
  motors.setSpeeds(-rightSpeed, -leftSpeed);

  if(headCountsLeft-250>encoders.getCountsLeft() && headCountsRight-250>encoders.getCountsRight()){
    mode=2;
  }
}

void trash2(){
  //Mode 6
delay(550);

  while(angle > heading - 45){
    Angle();
    
    leftSpeed = 150;
    
    rightSpeed = - 150;
    
    motors.setSpeeds(leftSpeed, rightSpeed);
  }

  headCountsLeft = encoders.getCountsLeft();
  headCountsRight = encoders.getCountsRight(); 

  display.clear();

  heading = angle;

  heading = angle;

  while(angle < heading + 135){
    Angle();
    
    leftSpeed = 80;
    
    rightSpeed = 400;
    
    motors.setSpeeds(leftSpeed, rightSpeed);
  }

  heading = angle;

  while(true){
    Angle();
    error = heading - angle;
    leftSpeed = 350 - error*6;
    rightSpeed = 350 + error*6;
    motors.setSpeeds(leftSpeed, rightSpeed);

    lineSensors.read(lineSensorValues, useEmitters ? QTR_EMITTERS_ON : QTR_EMITTERS_OFF);
    leftValue = lineSensorValues[0];
    mLeftValue = lineSensorValues[1];
    midValue = lineSensorValues[2];
    mRightValue = lineSensorValues[3];
    rightValue = lineSensorValues[4];
    lineSensors.emittersOn();

    if(leftValue<300|mLeftValue<300|midValue<300|mRightValue<300|rightValue<300){
      mode=7;
      break;
    }
  }
}

void reverse2(){
  //Mode 7

  headCountsLeft = encoders.getCountsLeft();
  headCountsRight = encoders.getCountsRight(); 
  heading = angle;

  while(headCountsLeft-400<encoders.getCountsLeft() && headCountsRight-400<encoders.getCountsRight()){
    Angle();
    error = heading - angle;
    leftSpeed = 300 - error*6;
    rightSpeed = 300 + error*6;
    motors.setSpeeds(-rightSpeed,-leftSpeed);
  }

  while(angle > heading - 89){
    Angle();
    
    leftSpeed = -80;
    
    rightSpeed = -400;
    
    motors.setSpeeds(leftSpeed, rightSpeed);
  }

  headCountsLeft = encoders.getCountsLeft();
  headCountsRight = encoders.getCountsRight(); 

  display.clear();

  heading = angle;

  while(headCountsLeft-1300<encoders.getCountsLeft() && headCountsRight-1300<encoders.getCountsRight()){
    Angle();
    error = heading - angle;
    leftSpeed = 350 - error*6;
    rightSpeed = 350 + error*6;
    motors.setSpeeds(-rightSpeed,-leftSpeed);
  }
  mode=2;
}

void loop() {
  Angle();

  if (mode!=3){
  lineSensors.read(lineSensorValues, useEmitters ? QTR_EMITTERS_ON : QTR_EMITTERS_OFF);
  leftValue = lineSensorValues[0];
  mLeftValue = lineSensorValues[1];
  midValue = lineSensorValues[2];
  mRightValue = lineSensorValues[3];
  rightValue = lineSensorValues[4];
  lineSensors.emittersOn();
  }

  switch(mode){
  case 1:
    finder(leftValue,mLeftValue,midValue,mRightValue,rightValue);
    break;
  case 2:
    lineFollow(leftValue,mLeftValue,midValue,mRightValue,rightValue);
    break;
  case 3:
    detect();
    break;
  case 4:
    trash1(leftValue,mLeftValue,midValue,mRightValue,rightValue);
    break;
  case 5:
    reverse1();
    break;
  case 6:
    trash2();
    break;
  case 7:
    reverse2();
    break;
  }

  Serial.print(" MODE: "); 
  Serial.println(mode); 

  display.gotoXY(0, 1);
  display.print(mode);

  display.gotoXY(6, 1);
  display.print(angle);
}
