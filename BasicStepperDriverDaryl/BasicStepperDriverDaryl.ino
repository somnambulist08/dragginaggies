/* Via the magic of copy and paste, this worked and flew on December 16, 2023
 * Derived from the 795 line behemoth of aldebaran_basic.ino
 */


/*
 * Simple demo, should work with any driver board
 *
 * Connect STEP, DIR as indicated
 *
 * Copyright (C)2015-2017 Laurentiu Badea
 *
 * This file may be redistributed under the terms of the MIT license.
 * A copy of this license has been included with this distribution in the file LICENSE.
 */
#include <Arduino.h>
#include "BasicStepperDriver.h"
#include <math.h>
#include <SD.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BMP280.h>


// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
#define MOTOR_STEPS 200
#define RPM 150



#define DTF
// Since microstepping is set externally, make sure this matches the selected mode
// If it doesn't, the motor will move at a different RPM than chosen
// 1=full step, 2=half step etc.
#define MICROSTEPS 8

// All the wires needed for full functionality
#define DIR 8
#define STEP 9

#define CW 7
#define CCW 6

int currentStep = 0;
// 2-wire basic config, microstepping is hardwired on the driver
BasicStepperDriver stepper(MOTOR_STEPS, DIR, STEP);

float groundPressure = 0.0;
float t_prev = 0.0;
float t_now = 0.0;
float acc_x = 0.0;
float g_metric = 0.0;
float angleCommand = 0.0;
//External Sensor variables
Adafruit_BMP280 external_barometer;
Adafruit_BNO055 external_IMU;
long ext_ID = 55;
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;

//Data Recording
File flightData;
String fileName;

int stepsFromAngle(float angle, unsigned int microStepping = 1) {
  angle -= 0.0964738;
  float A = 102.1838 - (31.144823 * cos(angle) + sqrt(5625.0 - pow(26.6192 + 31.144823 * sin(angle), 2)));
  float steps = A / 2.54 * 360.0 / 1.8 * microStepping;
  return (int)steps;
}
void moveToAngle(float ang){
  int steps = stepsFromAngle(ang, MICROSTEPS);
  int moveSteps = steps - currentStep;
  stepper.move(moveSteps);
  currentStep += moveSteps;
}

int expand = 0;
int retract = 0;
//Uncomment line to use enable/disable functionality
//#define SLEEP 13



//Uncomment line to use enable/disable functionality
//BasicStepperDriver stepper(MOTOR_STEPS, DIR, STEP, SLEEP);

void setup() {
  Serial.begin(9600);
    stepper.begin(RPM, MICROSTEPS);
    // if using enable/disable on ENABLE pin (active LOW) instead of SLEEP uncomment next line
    // stepper.setEnableActiveState(LOW);
  while (!Serial) {
    Serial.println("Waiting for serial...");
    delay(100);
  }


  //I2C sensors
  //setup bno055 imu @ 0x28
  external_IMU = Adafruit_BNO055(ext_ID, 0x28, &Wire);
  if (!external_IMU.begin(OPERATION_MODE_AMG)) {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1)
      ;
  }
  external_IMU.set16GRange();
  //setup bmp280 pressure/temp @0x77
  unsigned status;
  status = external_barometer.begin();
  //sensor is on, want 1x oversampling on temps, 16x oversample on pressure, no filter, minimal standby
  //rms noise: 0.005[C], 1.3[Pa]
  external_barometer.setSampling(external_barometer.MODE_NORMAL, external_barometer.SAMPLING_X1, external_barometer.SAMPLING_X16, external_barometer.FILTER_OFF, external_barometer.STANDBY_MS_1);
  
#ifdef DTF
  //setup SDcard reader
  Serial.print("Initializing SD card...");
  if (!SD.begin(10)) {
    Serial.println("initialization failed!");
    while (1)
      ;
  }
  Serial.println("initialization done.");
  //hippity hoppity this memory is now my property
  bool fileavail = false;
  String numb = "FNF";
  int i = 0;
  while (!fileavail) {  //less stupid naming convention - 0000.csv to 9999.csv
    if (i < 10)
      numb = String("000" + String(i) + ".csv");
    else if (i < 100)
      numb = String("00" + String(i) + ".csv");
    else if (i < 1000)
      numb = String("0" + String(i) + ".csv");
    else
      numb = String(String(i) + ".csv");
    fileavail = !SD.exists(numb);
    i++;
  }
  flightData = SD.open(numb, FILE_WRITE);

  // if the file opened okay, write to it:
  if (flightData) {
    Serial.print("Writing to " + numb);  //Only in AMG mode this time
    //flightData.println("Time[ms], Temp[C], Pressure[Pa]");
    flightData.print("Time[ms], Temp(baro)[C], Pressure[Pa],");
    flightData.print(" Omega1[rad/s], Omega2[rad/s], Omega3[rad/s], acc1[m/s2], acc2[m/s2], acc3[m/s2],");
    flightData.print(" mag1[uT], mag2[uT], mag3[uT],");
    //calculated values and sent commands
    flightData.println(" vertV[m/s], predictedApogee[m], desiredApogee[m], flapAngle[rad], stepperPos");
    Serial.println("...headers done.");
    fileName = flightData.name();
    flightData.close();
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening " + numb);
    //red light means stop
    while (1)
      ;
  }
#endif
  groundPressure = external_barometer.readPressure();
  t_prev = 0;
  t_now = micros();
  imu::Vector<3> totalacc = external_IMU.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  acc_x = totalacc.x() + g_metric;
  Serial.println("Startup done...");
}


long accelerint(float a0, float a1, float dt) {
  return dt * (a1 + a0) / 2.0;
}
float pressureAlt(float pressure) {
  return (1 - powf((pressure / 101325), 0.190284)) * 145366.45 * 0.3048;
}
float getDesired(float time) {
  return 480.0f * powf(M_E, -time * 3.0f / 10.0f) + 1143.0f;
}

float predictAltitude(float height, float velocity){
  return height + velocity*velocity/2/9.81;
}
 
  
#define P -0.0006
#define I -0.0006
 
float integratorState = 0;
float getControl(float desired, float predicted, float dt){
  float err = desired - predicted;
  float control = P * err;
  integratorState += I*err*dt;
 
  control += PI/6;
  if (control > PI/2) control = PI/2;
  if (control < 0) control = 0;
  return control;
}

long unsigned burnout_time = 0;
float timeSinceBurnout=0;
float lastStep=0;
float timeAtBurnout=0;
float ang = 0;

bool launched = false;
bool burnout = false;
bool apogee = false;
bool startedup = false;


float acc_x_t1 = 0;
float velocity = 0;


void loop() {
  //retract = 1;
  if (!startedup) {
    flightData = SD.open(fileName, FILE_WRITE);
    startedup = true;
  }
  float temp = external_barometer.readTemperature();
  //pressure
  float pressure = external_barometer.readPressure();
  acc_x_t1 = acc_x;
  imu::Vector<3> gyro = external_IMU.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> totalacc = external_IMU.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> magn = external_IMU.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

  acc_x = totalacc.x() + g_metric;

  //acc_x = tableInterpAccel(t_now/ 1000000.0f);

  t_prev = t_now;
  t_now = micros();



  float dt = (t_now - t_prev) / 1000000.0f;
  velocity += accelerint(acc_x_t1, acc_x, dt);
//  velocity = 160.0; //god fucking dammit this is why that didn't work in flight dec 15
  float altitude = pressureAlt(pressure) - pressureAlt(groundPressure);
  //altitude = tableInterpHeight(t_now/ 1000000.0f);
  if (acc_x >= 50) {
    delay(100);
    temp = external_barometer.readTemperature();
    //pressure
    pressure = external_barometer.readPressure();
    acc_x_t1 = acc_x;
    imu::Vector<3> gyro = external_IMU.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu::Vector<3> totalacc = external_IMU.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    imu::Vector<3> magn = external_IMU.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
    acc_x = totalacc.x() + g_metric;
    
    //acc_x = tableInterpAccel(t_now/ 1000000.0f);
    t_prev = t_now;
    t_now = micros();
    dt = (t_now - t_prev) / 1000000.0f;
    velocity += accelerint(acc_x_t1, acc_x, dt);
    altitude = pressureAlt(pressure) - pressureAlt(groundPressure);
    //altitude = tableInterpHeight(t_now/ 1000000.0f);
    
    launched=(acc_x>=50);
    //Serial.println("YEET YEET");
  }
  if(launched && !burnout){
    burnout = (acc_x<0);
    if(burnout)
      timeAtBurnout=micros();
    Serial.println("NO MORE YEET");
  }
  if(launched && burnout && !apogee){
    //DO SOME SHIT WITH THE FLAPS
  timeSinceBurnout = micros() - timeAtBurnout;
  ang = getControl(getDesired(timeSinceBurnout), predictAltitude(altitude, velocity), timeSinceBurnout-lastStep);
  lastStep = timeSinceBurnout;
  Serial.print("Moving to angle ");
  Serial.println(ang);
  moveToAngle(-ang);
  apogee = (velocity <= 0);
  }
  
  if(launched && burnout && apogee)
  {
    Serial.println("Apogee detected cookie factory's closed");
    moveToAngle(0);
    flightData.close();
    flightData = SD.open(fileName, FILE_WRITE);
  }

#ifdef DTF

  flightData.print(String(t_now) + ',' + String(temp) + ',' + String(pressure) + ',');
  flightData.print(String(gyro.x()) + ',' + String(gyro.y()) + ',' + String(gyro.z()) + ',');
  flightData.print(String(totalacc.x()) + ',' + String(totalacc.y()) + ',' + String(totalacc.z()) + ',' + String(magn.x()) + ',' + String(magn.y()) + ',' + String(magn.z()) + ',');
  flightData.println(String(velocity) + ','+String(getDesired(timeSinceBurnout))+','+ String(predictAltitude(altitude, velocity))+','+String(ang));
  //flightData.println(String(predicted_apogee) + ',' + String(desired_apogee) + ',' + String(drag_flap_angle) + ',' + String(currentStep) + '\n');
  flightData.flush();
#endif
   // expand = digitalRead(CW);
   // retract = digitalRead(CCW);
   //moveToAngle(PI/4);
   // delay(500);
   //moveToAngle(0);
  //retract=1;
    // energize coils - the motor will hold position
    // stepper.enable();

    if(expand){
      Serial.println("OUT");
      stepper.move(5*16);
      expand = 0;
    }
    else if(retract){      
      Serial.println("BACK");
      stepper.move(-5*16);
      retract = 0;
    }
    /*
     * Moving motor one full revolution using the degree notation
     */
/*    while(expand){
      stepper.rotate(1);
      stepper.move(50*8);
      
      expand=digitalRead(CW);
    }
    while(retract){
      stepper.rotate(-1);
      stepper.move(-50*8);
      delay(125);
      retract=digitalRead(CCW);
    }*/
    /*
     * Moving motor to original position using steps
     */
//    

    // pause and allow the motor to be moved by hand
    // stepper.disable();

//    delay(5000);
}
