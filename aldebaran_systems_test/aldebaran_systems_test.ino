#include <math.h>
#include <Arduino.h>
#include "BasicStepperDriver.h"
//external cards
#include <SD.h>
//internal sensors
#include "Arduino_BMI270_BMM150.h"  //IMU
#include <Arduino_HS300x.h>         //temp and humidity
#include <Arduino_LPS22HB.h>        //pressure
#include <ArduinoBLE.h>
#include <BLEStringCharacteristic.h>
 
//#define wired //comment this line out to use bluetooth

#define MOTOR_STEPS 200 //steps per rev
int RPM = 120; //speed? 
int MICROSTEPS = 8; 
//MS1 and MS2 are open so locked to 8
//psych! not anymore

// wat pins doing??? 
// MS2  MS1 microsteps
// GND  GND 8
// GND  VIO 32
// VIO  GND 64
// VIO  VIO 16

#define G_MET 9.81
#define G_IMP 32.174

//pins.
const pin_size_t RED =22;
const pin_size_t GREEN =23;
const pin_size_t BLUE =24;     //onboard LED

const pin_size_t MS1 = 4;
const pin_size_t MS2 = 5; // controls for microstepping

const pin_size_t BZZT =6;
const pin_size_t EN =7;
const pin_size_t STEP =8;
const pin_size_t DIRECTION =9;

const pin_size_t wireExpand =2;
const pin_size_t wireRetract =3;


int expand = 1;
int retract = 1;
int currentStep = 0;


//yes, apparently this is (all) required to flip one bit in a register
class MyBoschSensor : public BoschSensorClass {
public:
  MyBoschSensor(TwoWire &wire = Wire)
    : BoschSensorClass(wire){};

protected:
  virtual int8_t configure_sensor(struct bmi2_dev *dev) {
    int8_t rslt;
    uint8_t sens_list[2] = { BMI2_ACCEL, BMI2_GYRO };

    struct bmi2_int_pin_config int_pin_cfg;
    int_pin_cfg.pin_type = BMI2_INT1;
    int_pin_cfg.int_latch = BMI2_INT_NON_LATCH;
    int_pin_cfg.pin_cfg[0].lvl = BMI2_INT_ACTIVE_HIGH;
    int_pin_cfg.pin_cfg[0].od = BMI2_INT_PUSH_PULL;
    int_pin_cfg.pin_cfg[0].output_en = BMI2_INT_OUTPUT_ENABLE;
    int_pin_cfg.pin_cfg[0].input_en = BMI2_INT_INPUT_DISABLE;

    struct bmi2_sens_config sens_cfg[2];
    sens_cfg[0].type = BMI2_ACCEL;
    sens_cfg[0].cfg.acc.bwp = BMI2_ACC_OSR2_AVG2;
    sens_cfg[0].cfg.acc.odr = BMI2_ACC_ODR_25HZ;
    sens_cfg[0].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;
    sens_cfg[0].cfg.acc.range = BMI2_ACC_RANGE_16G;
    sens_cfg[1].type = BMI2_GYRO;
    sens_cfg[1].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;
    sens_cfg[1].cfg.gyr.bwp = BMI2_GYR_OSR2_MODE;
    sens_cfg[1].cfg.gyr.odr = BMI2_GYR_ODR_25HZ;
    sens_cfg[1].cfg.gyr.range = BMI2_GYR_RANGE_2000;
    sens_cfg[1].cfg.gyr.ois_range = BMI2_GYR_OIS_2000;

    rslt = bmi2_set_int_pin_config(&int_pin_cfg, dev);
    if (rslt != BMI2_OK)
      return rslt;

    rslt = bmi2_map_data_int(BMI2_DRDY_INT, BMI2_INT1, dev);
    if (rslt != BMI2_OK)
      return rslt;

    rslt = bmi2_set_sensor_config(sens_cfg, 2, dev);
    if (rslt != BMI2_OK)
      return rslt;

    rslt = bmi2_sensor_enable(sens_list, 2, dev);
    if (rslt != BMI2_OK)
      return rslt;

    return rslt;
  }
};
MyBoschSensor sixteenIMU(Wire1);




//Data Recording
File flightData;
String fileName;

float acc_int_x = 0.0f, 
      acc_int_y = 0.0f, 
      acc_int_z = 0.0f;
float gyro_int_x = 0.0f, 
      gyro_int_y = 0.0f, 
      gyro_int_z = 0.0f;
float mag_int_x = 0.0f, 
      mag_int_y = 0.0f, 
      mag_int_z = 0.0f;

float temp_int = 0.0f;
float pressure_int = 0.0f;

//calculated and derived values
unsigned long t_prev = -1;
unsigned long t_now = 0;
float v_int_x = 0.0f;
float v_int_y = 0.0f;
float v_int_z = 0.0f;

float groundPressure;
float minburn;
float expected_max_acc;

float timeSinceBurnout = 0.0f;
float predicted_apogee = 0.0f;
float desired_apogee = 0.0f;

//TMC2226 is a TMC2209 in a different package

BasicStepperDriver stepper(MOTOR_STEPS, DIRECTION, STEP, EN);

#ifndef MAX_BLE_STR_LEN
#define MAX_BLE_STR_LEN 100
#endif


#ifndef PI
#define PI 3.14159265358979323846
#endif
 
BLEService stepperService("19B10000-E8F2-537E-4F6C-D104768A1214"); // create services
//BLEService ledService("19B10001-E8F2-537E-4F6C-D104768A1216");
//BLEService buzzService("19B10001-E8F2-537E-4F6C-D104768A1217");
BLEService microstepsService("19B10001-E8F2-537E-4F6C-D104768A1269");
BLEService ppsService("19B10001-E8F2-537E-4F6C-D080085A1269");
 
// create switch characteristic and allow remote device to read and write
BLEStringCharacteristic stepperAngle("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite, MAX_BLE_STR_LEN);
BLEStringCharacteristic stepperDegOrRad("19B10001-E8F2-537E-4F6C-D104768A1215", BLERead | BLEWrite, MAX_BLE_STR_LEN);
BLEStringCharacteristic microstepsStr("19B10001-E8F2-537E-4F6C-D104768A1269", BLERead | BLEWrite, MAX_BLE_STR_LEN);
BLEStringCharacteristic pps("19B10001-E8F2-537E-4F6C-D080085A1269", BLERead | BLEWrite, MAX_BLE_STR_LEN);
// allow remote device to flash LED and hit buzzer
//BLEByteCharacteristic LED("LEDBLINK-E8F2-537E-4F6C-D104768A1216", BLERead | BLEWrite);
//BLEByteCharacteristic buzz("19B10001-E8F2-537E-4F6C-D104768A1217", BLERead | BLEWrite);

void bluetoothSetup();
 
void blePeripheralConnectHandler(BLEDevice central);
void blePeripheralDisconnectHandler(BLEDevice central);
void stepperAngleWritten(BLEDevice central, BLECharacteristic characteristic);
void stepperDegOrRadWritten(BLEDevice central, BLECharacteristic characteristic);
void buzzWritten(BLEDevice central, BLECharacteristic characteristic);
void ledWritten(BLEDevice central, BLECharacteristic characteristic);
void microstepsWritten(BLEDevice central, BLECharacteristic characteristic);
void ppsWritten(BLEDevice central, BLECharacteristic characteristic);

void stringToByteArray(const char* str, uint8_t* bArr, size_t lenStr);
void byteArrayToString(uint8_t* bArr, char* str, size_t lenBArr);
 
bool degMode = true;
float angleCommand_rad = 0.0; 
 
void bluetoothSetup(){
  // begin initialization
  if (!BLE.begin()) {
    Serial.println("starting Bluetooth® Low Energy module failed!");
    digitalWrite(RED,HIGH);
    while (1);
  }
  Serial.println("Bluetooth Module Started");
 
  // set the local name peripheral advertises
  BLE.setLocalName("Aldebaran");
  // set the UUID for the service this peripheral advertises
  BLE.setAdvertisedService(stepperService);
//  BLE.setAdvertisedService(ledService);
//  BLE.setAdvertisedService(buzzService);
  BLE.setAdvertisedService(microstepsService);
  BLE.setAdvertisedService(ppsService);

  // add the characteristic to the service
  stepperService.addCharacteristic(stepperAngle);
  stepperService.addCharacteristic(stepperDegOrRad);
  //ledService.addCharacteristic(LED);
  //buzzService.addCharacteristic(buzz);
  microstepsService.addCharacteristic(microstepsStr);
  ppsService.addCharacteristic(pps);

  // add service
  BLE.addService(stepperService);
  //BLE.addService(ledService);
  //BLE.addService(buzzService);
  BLE.addService(microstepsService);
  BLE.addService(ppsService);

  // assign event handlers for connected, disconnected to peripheral
  BLE.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  BLE.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);
 
  // assign event handlers for characteristic
  stepperDegOrRad.setEventHandler(BLEWritten, stepperDegOrRadWritten);
  stepperAngle.setEventHandler(BLEWritten, stepperAngleWritten);
  //LED.setEventHandler(BLEWritten, ledWritten);
  //buzz.setEventHandler(BLEWritten, buzzWritten);
  microstepsStr.setEventHandler(BLEWritten, microstepsWritten);
  pps.setEventHandler(BLEWritten, ppsWritten);

  // set an initial value for the characteristic
  stepperDegOrRad.writeValue("deg");
  stepperDegOrRad.writeValue("0.000");
  //LED.writeValue(0);
  //buzz.writeValue(0);
  microstepsStr.writeValue("8");
  pps.writeValue("1600");
  // start advertising
  BLE.advertise();

 
  Serial.println(("Bluetooth® device active, waiting for connections..."));
}
 
void blePeripheralConnectHandler(BLEDevice central) {
  // central connected event handler
  Serial.print("Connected event, central: ");
  Serial.println(central.address());
}
void blePeripheralDisconnectHandler(BLEDevice central) {
  // central disconnected event handler
  Serial.print("Disconnected event, central: ");
  Serial.println(central.address());
}
void ppsWritten(BLEDevice central, BLECharacteristic characteristic){
  Serial.println("steps per sec written:");
  Serial.println(pps.value());
  RPM = MOTOR_STEPS*MICROSTEPS*60*pps.value().toInt();
}
void ledWritten(BLEDevice central, BLECharacteristic characteristic){
  Serial.print("FRANZ BLINK DER BLINKENLIGHTS AUF BLAUTOOFEN");
  digitalWrite(BLUE,HIGH);
  Serial.print('\n');
}
void buzzWritten(BLEDevice central, BLECharacteristic characteristic){
  Serial.print("ha ha buzzer goes brrr...with bluetooth");
  digitalWrite(BZZT,HIGH);
  delay(250);
  digitalWrite(BZZT,LOW);
  Serial.print('\n');
}
// wat pins doing??? 
// MS2  MS1 microsteps
// GND  GND 8
// GND  VIO 32
// VIO  GND 64
// VIO  VIO 16

void microstepsWritten(BLEDevice central, BLECharacteristic characteristic){
  Serial.print("microstepper go brrrr:");
  Serial.print('\n');
  Serial.println(microstepsStr.value());
  switch(microstepsStr.value().toInt()){
    case 64:
    digitalWrite(MS2,HIGH);
    digitalWrite(MS1,LOW);
    MICROSTEPS = 64;
    break;
    case 32:
    digitalWrite(MS2,LOW);
    digitalWrite(MS1,HIGH);
    MICROSTEPS = 32;
    break;
    case 16:
    digitalWrite(MS2,HIGH);
    digitalWrite(MS1,HIGH);
    MICROSTEPS = 16;
    break;
    default:
    digitalWrite(MS2,LOW);
    digitalWrite(MS1,LOW);
    MICROSTEPS = 8;
  }
  Serial.println("microsteps set to: "+String(MICROSTEPS));
}  
void stepperAngleWritten(BLEDevice central, BLECharacteristic characteristic) {
  // central wrote a new value to the characteristic, update LED
  Serial.print("Characteristic event, Stepper Angle written: ");
  Serial.print(stepperAngle.value());
  Serial.print('\n');
 
  String angString = stepperAngle.value();
 
  float ang = angString.toFloat();  // Convert the string to a floating-point number
 
  char buffer[50];
  if (degMode) {
    if (ang >= 0.0 && ang <= 90.0) {
      sprintf(buffer, "Accepted value: %f deg", ang);
      Serial.println(buffer);
      stepperAngle.setValue(String(ang)); // the characteristic is stored in degrees
      angleCommand_rad = PI / 360.0 * ang; // while the command is always stored in rad
    } else {
      Serial.println("Angle not accepted");
    }
  } else {
    if (ang >= 0.0 && ang <= PI / 2) {
      sprintf(buffer, "Accepted value: %f rad", ang);
      Serial.println(buffer);
      stepperAngle.setValue(String(ang));
      angleCommand_rad = ang;
    } else {
      Serial.println("Angle not accepted");
    }
  }
} 
void stepperDegOrRadWritten(BLEDevice central, BLECharacteristic characteristic) {
  Serial.print("Characteristic event, Stepper Mode written: ");
  Serial.print(stepperDegOrRad.value());
  Serial.print('\n');
 
  String str = stepperDegOrRad.value();
 
  if( str[0] == 'd' || str[0] == 'D' ) {
    if(degMode){
      Serial.println("Already in degree input mode, no change");
    } else {
      degMode = true;
      Serial.println("Switching from radian input to degree input. Update stepper command to continue.");
    }
  } else {
    if (str[0] == 'r' || str[0] == 'R') {
      if(degMode){
        Serial.println("Switching from degree input to radian input. Update stepper command to continue.");
        degMode = false;
      } else {
        Serial.println("Already in radian input mode, no change");
      }
    }
  }
}

/**
* Given a flap angle, this function converts it to the needed step position.
*/
int stepsFromFlapAngle(float angle) {
  angle -= 0.0964738;
  float A = 102.1838 - (31.144823 * cos(angle) + sqrt(5625.0 - pow(26.6192 + 31.144823 * sin(angle), 2)));
  float steps = A / 2.54 * 360.0 / 1.8;
  return (int)steps;
}

void setup() {
  pinMode(RED, OUTPUT); //Debug LED and buzzer
  pinMode(GREEN, OUTPUT);
  pinMode(BLUE, OUTPUT);
  pinMode(BZZT, OUTPUT);
  pinMode(MS1, OUTPUT);
  pinMode(MS2, OUTPUT);
  digitalWrite(MS1,LOW);
  digitalWrite(MS2,LOW);
  digitalWrite(BLUE,LOW);
  digitalWrite(GREEN,LOW);
  digitalWrite(RED,LOW);


  //Serial link initialization
  Serial.begin(9600);  //baud rate goes brrrrr
  while (!Serial) {
    Serial.println("Waiting for serial...");
    delay(100);
  }

  bluetoothSetup();
  digitalWrite(BLUE,HIGH);
  delay(500);
  digitalWrite(BLUE,LOW);

  pinMode(wireExpand,INPUT);
  pinMode(wireRetract,INPUT);

  stepper.begin(RPM, MICROSTEPS);
  stepper.setEnableActiveState(LOW);
  //onboard IMU: 16g range, 2000dps gyro
  if (!sixteenIMU.begin()) {  //startup
    Serial.println("Failed to initialize internal IMU!");
    digitalWrite(RED, HIGH);
    while (1);
  }//onboard thermometer
  if (!HS300x.begin()) {
    Serial.println("Failed to initialize internal thermometer!");
    digitalWrite(RED, HIGH);
    while (1);
  } //onboard pressure
  if (!BARO.begin()) {
    Serial.println("Failed to initialize internal pressure sensor!");
    digitalWrite(RED, HIGH);
    while (1);
  }  
  //setup SDcard reader
/*  Serial.print("Initializing SD card...");
  if (!SD.begin(10)) {
    Serial.println("SD init failed!");
    while (1) {
      digitalWrite(RED, HIGH);
      delay(1000);
      digitalWrite(RED, LOW);
      delay(1000);
    }
  }
  Serial.println("SD init done.");

  //hippity hoppity this memory is now my property
  bool fileavail = false;
  String numb = "FNF";
  int i = 0;
  while (!fileavail) { //less stupid naming convention - 0000.csv to 9999.csv
    if (i < 10) 
      numb = String("000" + String(i) + ".csv");
    else if (i < 100) 
      numb = String("00" + String(i) + ".csv");
    else if (i < 1000)
      numb = String("0" + String(i) + ".csv");
    else
      numb = String(String(i)+".csv");
    fileavail = !SD.exists(numb);
    i++;
  }
  flightData = SD.open(numb, FILE_WRITE);
  
  // if the file opened okay, write to it:
  if (flightData) {
    Serial.print("Writing to " + numb);
    //Onboard Sensors Start
    flightData.print("Time[micros], ");
    flightData.print(" Temp[C], Pressure[kPa],");
    flightData.print(" Omega1[dps], Omega2[dps], Omega3[dps], acc1[g], acc2[g], acc3[g],");
    flightData.print(" mag1[uT], mag2[uT], mag3[uT]");
    //Calculated/Commanded
    flightData.println(" vertV[m/s], predictedApogee[m], desiredApogee[m], flapAngle[rad], stepperPos");
    Serial.println("...headers done.");
    fileName = flightData.name();
    flightData.close();
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening " + numb);
    //red light means stop
    while (1) {
      digitalWrite(RED, HIGH);
      delay(1000);
      digitalWrite(RED, LOW);
      delay(1000);
    }
  }
  */
  
  t_prev = micros();
  digitalWrite(GREEN,HIGH);
  digitalWrite(BLUE,HIGH);
  digitalWrite(RED,HIGH); //all lights for go
  delay(500);
  digitalWrite(GREEN,LOW);
  digitalWrite(BLUE,LOW);
  digitalWrite(RED,LOW); //all lights for go
  delay(500);
  digitalWrite(GREEN,HIGH);
  digitalWrite(BLUE,HIGH);
  digitalWrite(RED,HIGH); //all lights for go
  delay(500);
  digitalWrite(GREEN,LOW);
  digitalWrite(BLUE,LOW);
  digitalWrite(RED,LOW); //all lights for go
  delay(500);
  digitalWrite(GREEN,HIGH);
  digitalWrite(BLUE,HIGH);
  digitalWrite(RED,HIGH); //all lights for go
  delay(500);
  digitalWrite(GREEN,LOW);
  digitalWrite(BLUE,LOW);
  digitalWrite(RED,LOW); //all lights for go

  digitalWrite(BZZT,HIGH);
  delay(500);
  digitalWrite(BZZT,LOW);
  stepper.enable();
  Serial.println("Startup done...");
}


void loop() {
  digitalWrite(GREEN,1);
  digitalWrite(BLUE,1);
  digitalWrite(RED,1);
  t_prev = t_now;
  t_now = micros();
  if (sixteenIMU.accelerationAvailable()) {
    sixteenIMU.readAcceleration(acc_int_x, acc_int_y, acc_int_z);
  }
  if (sixteenIMU.gyroscopeAvailable()) {
    sixteenIMU.readGyroscope(gyro_int_x, gyro_int_y, gyro_int_z);
  }
  if (sixteenIMU.magneticFieldAvailable()){
    sixteenIMU.readMagneticField(mag_int_x,mag_int_y,mag_int_z);
  }
  temp_int = HS300x.readTemperature();
  pressure_int = BARO.readPressure();
  int RPMold = RPM;
  int MICROSold=MICROSTEPS;
  BLE.poll();
  if(RPM != RPMold || MICROSold != MICROSTEPS){
    stepper.begin(RPM,MICROSTEPS);
    stepper.enable();
  }
  char buffer[50];
  int neededSteps = (stepsFromFlapAngle(angleCommand_rad)- currentStep)*MICROSTEPS;
  if (neededSteps != 0) {
    sprintf(buffer, "OLDcurrentStep*MICROSTEPS: %i", currentStep*MICROSTEPS);
    Serial.println(buffer);
    sprintf(buffer, "neededSteps: %i", neededSteps);
    Serial.println(buffer);
    stepper.move(-neededSteps);
    currentStep += neededSteps/MICROSTEPS;
    sprintf(buffer, "NEWcurrentStep*MICROSTEPS: %i", currentStep*MICROSTEPS);
    Serial.println(buffer);
    neededSteps = 0;
  }
  else{
    retract=digitalRead(wireExpand);
    expand=digitalRead(wireRetract);
    while (expand == 1){
      Serial.println("Moving out...");
      digitalWrite(BLUE,0);
      stepper.move(MICROSTEPS);
      //delay(10);
      expand=digitalRead(wireRetract);
    }
    while (retract == 1){
      Serial.println("Returning.");
      digitalWrite(RED,0);
      stepper.move(-MICROSTEPS);
      //delay(10);
      retract=digitalRead(wireExpand);
    }
  }
  /*
  flightData.print(String(t_now) + ',' + String(temp_int) + ',' + String(pressure_int) + ',');
  flightData.print(String(gyro_int_x) + ',' + String(gyro_int_y) + ',' + String(gyro_int_z) + ',');
  flightData.print(String(acc_int_x) + ',' + String(acc_int_y) + ',' + String(acc_int_z) + ',' + String(mag_int_x) + ',' + String(mag_int_y) + ',' + String(mag_int_z) + ',');
  //flightData.println(String(v_int_x) + ','+String(getDesired(timeSinceBurnout))+','+ String(predictAltitude(altitude, velocity))+','+String(ang)+','+String(currentStep));
  flightData.println(String(v_int_x) + ',' + String(predicted_apogee) + ',' + String(desired_apogee) + ',' + String(angleCommand_rad) + ',' + String(currentStep) + '\n');
  flightData.flush();*/
}
