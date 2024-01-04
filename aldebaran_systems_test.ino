#include <math.h>
#include <Arduino.h>
#include "BasicStepperDriver.h"
//external cards
#include <SD.h>
#include <Adafruit_BNO055.h> //ext. imu 
#include <Adafruit_BMP280.h> //ext pressure/temp
//internal sensors
#include "Arduino_BMI270_BMM150.h"  //IMU
#include <Arduino_HS300x.h>         //temp and humidity
#include <Arduino_LPS22HB.h>        //pressure
#include <ArduinoBLE.h>
#include <BLEStringCharacteristic.h>
 
#define wired //comment this line out to use bluetooth

#define MOTOR_STEPS 200 //steps per rev
#define RPM 45 //speed? 
#define MICROSTEPS 8 //MS1 and MS2 are open so locked to 8

#define G_MET 9.81
#define G_IMP 32.174


//pins.
const pin_size_t RED =22;
const pin_size_t GREEN =23;
const pin_size_t BLUE =24;     //onboard LED



const pin_size_t BZZT =6;
const pin_size_t EN =7;
const pin_size_t STEP =8;
const pin_size_t DIRECTION =9;

const pin_size_t wireExpand =2;
const pin_size_t wireRetract =3;


int expand = 1;
int retract = 1;
int currentStep = 0;


//External Sensor
Adafruit_BMP280 external_barometer;
Adafruit_BNO055 external_IMU;
long ext_ID = 55;
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;

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
float acc_ext_x = 0.0f, 
      acc_ext_y = 0.0f,
      acc_ext_z = 0.0f;
float gyro_int_x = 0.0f, 
      gyro_int_y = 0.0f, 
      gyro_int_z = 0.0f;
float gyro_ext_x = 0.0f, 
      gyro_ext_y = 0.0f,
      gyro_ext_z = 0.0f;
float mag_int_x = 0.0f, 
      mag_int_y = 0.0f, 
      mag_int_z = 0.0f;
float mag_ext_x = 0.0f, 
      mag_ext_y = 0.0f,
      mag_ext_z = 0.0f;

float temp_int = 0.0f;
float temp_ext = 0.0f;
float pressure_int = 0.0f;
float pressure_ext = 0.0f;

//calculated and derived values
unsigned long t_prev = -1;
unsigned long t_now = 0;
float v_int_x = 0.0f;
float v_int_y = 0.0f;
float v_int_z = 0.0f;

float groundPressure;
float minburn;
float expected_max_acc;

//TMC2226 is a TMC2209 in a different package

BasicStepperDriver stepper(MOTOR_STEPS, DIRECTION, STEP, EN);

#ifndef MAX_BLE_STR_LEN
#define MAX_BLE_STR_LEN 100
#endif


#ifndef PI
#define PI 3.14159265358979323846
#endif
 
BLEService stepperService("19B10000-E8F2-537E-4F6C-D104768A1214"); // create services
BLEService blinkService("19B10001-E8F2-537E-4F6C-D104768A1216");
BLEService buzzService("19B10001-E8F2-537E-4F6C-D104768A1217");
 
// create switch characteristic and allow remote device to read and write
BLEStringCharacteristic stepperAngle("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite, MAX_BLE_STR_LEN);
BLEStringCharacteristic stepperDegOrRad("19B10001-E8F2-537E-4F6C-D104768A1215", BLERead | BLEWrite, MAX_BLE_STR_LEN);
// allow remote device to flash LED and hit buzzer
BLEByteCharacteristic blink("19B10001-E8F2-537E-4F6C-D104768A1216", BLERead | BLEWrite);
BLEByteCharacteristic buzz("19B10001-E8F2-537E-4F6C-D104768A1217", BLERead | BLEWrite);

void bluetoothSetup();
 
void blePeripheralConnectHandler(BLEDevice central);
void blePeripheralDisconnectHandler(BLEDevice central);
void stepperAngleWritten(BLEDevice central, BLECharacteristic characteristic);
void stepperDegOrRadWritten(BLEDevice central, BLECharacteristic characteristic);
 
void stringToByteArray(const char* str, uint8_t* bArr, size_t lenStr);
void byteArrayToString(uint8_t* bArr, char* str, size_t lenBArr);
 
bool degMode = true;
float angleCommand_rad = 0.0; 
 
void bluetoothSetup(){
  // begin initialization
  if (!BLE.begin()) {
    Serial.println("starting Bluetooth® Low Energy module failed!");
 
    while (1);
  }
  Serial.println("Bluetooth Module Started");
 
  // set the local name peripheral advertises
  BLE.setLocalName("Aldebaran");
  // set the UUID for the service this peripheral advertises
  BLE.setAdvertisedService(stepperService);
  BLE.setAdvertisedService(blinkService);
  BLE.setAdvertisedService(buzzService);
 
  // add the characteristic to the service
  stepperService.addCharacteristic(stepperAngle);
  stepperService.addCharacteristic(stepperDegOrRad);
  blinkService.addCharacteristic(blink);
  buzzService.addCharacteristic(buzz);
 
  // add service
  BLE.addService(stepperService);
 
  // assign event handlers for connected, disconnected to peripheral
  BLE.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  BLE.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);
 
  // assign event handlers for characteristic
  stepperDegOrRad.setEventHandler(BLEWritten, stepperDegOrRadWritten);
  stepperAngle.setEventHandler(BLEWritten, stepperAngleWritten);
  // set an initial value for the characteristic
  stepperDegOrRad.writeValue("deg");
  stepperDegOrRad.writeValue("0.000");
  blink.writeValue(0);
  buzz.writeValue(0);
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
void blinkWritten(BLEDevice central, BLECharacteristic characteristic){
  Serial.print("FRANZ BLINK DER BLINKENLIGHTS AUF BLUTOOTHEN");
  Serial.print('\n');
}
void buzzWritten(BLEDevice central, BLECharacteristic characteristic){
  Serial.print("ha ha buzzer goes brrr...with bluetooth");
  Serial.print('\n');
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
* Warning: in both this and the simulink, the rounding can result in zero
* point shift in the presence of amplified noise. Maybe storing the needed
* step position as a float would be better and then only rounding when you
* actually push to the motor?
*/
int stepsFromFlapAngle(float newAngle){
  float theta = newAngle * 180.0/PI;
  float z = 2.636e-7*pow(theta, 4) + 4.879e-6*pow(theta, 3) - 0.007526*pow(theta, 2) - 0.1226*theta + 102.2;

  int steps = (int)(z/2.54*360.0/1.8);
  return steps;
}

void setup() {
  pinMode(RED, OUTPUT); //Debug LED and buzzer
  pinMode(GREEN, OUTPUT);
  pinMode(BLUE, OUTPUT);
  pinMode(BZZT, OUTPUT);
  digitalWrite(BLUE,LOW);
  digitalWrite(GREEN,LOW);
  digitalWrite(RED,LOW);
  #ifndef wired
  bluetoothSetup();
  digitalWrite(BLUE,HIGH);
  delay(500);
  digitalWrite(BLUE,LOW);
  #else
  pinMode(wireExpand,INPUT);
  pinMode(wireRetract,INPUT);
  #endif
  stepper.begin(RPM, MICROSTEPS);
  stepper.setEnableActiveState(LOW);
  //Serial link initialization
  Serial.begin(9600);  //baud rate goes brrrrr
  while (!Serial) {
    Serial.println("Waiting for serial...");
    delay(100);
  }
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

//I2C sensors
//setup bno055 imu @ 0x28
  external_IMU = Adafruit_BNO055(ext_ID, 0x28, &Wire);
  if (!external_IMU.begin(OPERATION_MODE_AMG)) { //only acc, mag, gyro - no default fusion for full accelerometer range
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    digitalWrite(RED,HIGH);
    while (1);
  }
  external_IMU.set16GRange(); 
    //setup bmp280 pressure/temp @0x77
  unsigned status;
  status = external_barometer.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
  //sensor is on, want 1x oversampling on temps, 16x oversample on pressure, no filter, minimal standby
  //rms noise: 0.005[C], 1.3[Pa]
  external_barometer.setSampling(external_barometer.MODE_NORMAL, external_barometer.SAMPLING_X1, external_barometer.SAMPLING_X16, external_barometer.FILTER_OFF, external_barometer.STANDBY_MS_1);


  
  //setup SDcard reader
  Serial.print("Initializing SD card...");
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
  /*
  // if the file opened okay, write to it:
  if (flightData) {
    Serial.print("Writing to " + numb);
    flightData.print("Time[ms], Tempex[C], Pressureex[Pa],");
    flightData.print(" Omega1ex[rad/s], Omega2ex[rad/s], Omega3ex[rad/s], acc1ex[m/s2], acc2ex[m/s2], acc3ex[m/s2],");
    flightData.print(" mag1ex[uT], mag2ex[uT], mag3ex[uT],");
    //Onboard Sensors Start
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
      digitalWrite(redpin, HIGH);
      delay(1000);
      digitalWrite(redpin, LOW);
      delay(1000);
    }
  }*/
  
  
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
  Serial.println("Startup done...");
}


void loop() {
  digitalWrite(GREEN,1);
  stepper.enable();
  if (sixteenIMU.accelerationAvailable()) {
    sixteenIMU.readAcceleration(acc_int_x, acc_int_y, acc_int_z);
  }
  if (sixteenIMU.gyroscopeAvailable()) {
    sixteenIMU.readGyroscope(gyro_int_x, gyro_int_y, gyro_int_z);
  }

  temp_int = HS300x.readTemperature();
  pressure_int = BARO.readPressure();

  #ifdef wired
  expand=digitalRead(wireExpand);
  retract=digitalRead(wireRetract);
  if (expand == 1){
    Serial.println("Moving out...");
    digitalWrite(BLUE,1);
    stepper.rotate(360);
    delay(100);
  }
  else{
    digitalWrite(BLUE,0);
  }
  if (retract == 1){
    Serial.println("Returning.");
    digitalWrite(RED,1);
    stepper.rotate(-360);
    delay(100);
  }
  else{
    digitalWrite(RED,0);
  }
  #else
  BLE.poll();
  char buffer[50];
  int neededSteps = (stepsFromFlapAngle(angleCommand_rad) - currentStep)*MICROSTEPS;
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
  #endif
}
