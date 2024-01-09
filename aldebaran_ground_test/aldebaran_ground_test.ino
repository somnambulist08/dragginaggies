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
 


#define RED 22     
#define BLUE 24     //onboard LED for debug code
#define GREEN 23

#define MOTOR_STEPS 200 //steps per rev
#define RPM 30 //speed? fix this later - marbe

#define MICROSTEPS 16

#define DIR 6
#define STEP 8


int expand = 0;
int retract = 0;
int currentStep = 0;

//External Sensor variables
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
unsigned long msSinceOn;

//calculated and derived values
unsigned long t_prev;
unsigned long t_now;
double v_int_x = 0;
double v_int_y = 0;
double v_int_z = 0;
static double g_metric = 9.81;
static double g_imp = 32.174;
float acc_x, acc_y, acc_z;
float mag_x, mag_y, mag_z;
float gyro_x, gyro_y, gyro_z;
float gyro_x_prev, gyro_y_prev, gyro_z_prev;
float acc_x_t1, acc_y_t1, acc_z_t1;
float acc_x_t0, acc_y_t0, acc_z_t0;

float groundPressure;
float minburn;
float expected_max_acc;

//TMC2226 is a TMC2209 in a different package

BasicStepperDriver stepper(MOTOR_STEPS, DIR, STEP);



#ifndef MAX_BLE_STR_LEN
#define MAX_BLE_STR_LEN 100
#endif
 
#ifndef PI
#define PI 3.14159265358979323846
#endif
 
BLEService stepperService("19B10000-E8F2-537E-4F6C-D104768A1214"); // create service
 
// create switch characteristic and allow remote device to read and write
BLEStringCharacteristic stepperAngle("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite, MAX_BLE_STR_LEN);
BLEStringCharacteristic stepperDegOrRad("19B10001-E8F2-537E-4F6C-D104768A1215", BLERead | BLEWrite, MAX_BLE_STR_LEN);
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
 
  // add the characteristic to the service
  stepperService.addCharacteristic(stepperAngle);
  stepperService.addCharacteristic(stepperDegOrRad);
 
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
  //float Lc = 75.0;
  //float a = 18.2638 *PI/180;
  //float Lf = 31.0;
 
  //float z = (Lc*cos(a) + Lf) - Lf*cos(newAngle)-sqrt(pow(Lc, 2) - pow(Lf*sin(newAngle) + Lc*sin(a), 2));

  //polynomial fit of theta_degrees to z_mm
  float theta = newAngle * 180.0/PI;
  float z = 2.636e-7*pow(theta, 4) + 4.879e-6*pow(theta, 3) - 0.007526*pow(theta, 2) - 0.1226*theta + 102.2;

  int steps = (int)(z/2.54*360.0/1.8);
  return steps;
}

void setup() {
  pinMode(RED, OUTPUT); //Debug LEDs
  pinMode(BLUE, OUTPUT);
  pinMode(GREEN, OUTPUT);
  bluetoothSetup();
  digitalWrite(BLUE,HIGH);
  delay(500);
  digitalWrite(BLUE,LOW);
  stepper.begin(RPM, MICROSTEPS);
  //Serial link initialization
  Serial.begin(9600);  //baud rate goes brrrrr
  while (!Serial) {
    Serial.println("Waiting for serial...");
    delay(100);
  }
  //onboard IMU: 16g range, 2000dps gyro
  //
  if (!sixteenIMU.begin()) {  //startup
    Serial.println("Failed to initialize IMU!");
    digitalWrite(RED, HIGH);
    while (1);
  }//onboard thermometer
  if (!HS300x.begin()) {
    Serial.println("Failed to initialize thermometer!");
    digitalWrite(RED, HIGH);
    while (1);
  } //onboard pressure
  if (!BARO.begin()) {
    Serial.println("Failed to initialize pressure sensor!");
    digitalWrite(RED, HIGH);
    while (1);
  }

//I2C sensors
//setup bno055 imu @ 0x28
  external_IMU = Adafruit_BNO055(ext_ID, 0x28, &Wire);
//  bno_write(BNO_ADDR, ACC_CONFIG, 0x0F); // accel +/-16g range (default value 0x0D)
  if (!external_IMU.begin()) {*/
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    digitalWrite(RED,HIGH);
    while (1);
  }
    //setup bmp280 pressure/temp @0x77
  unsigned status;
  status = external_barometer.begin();
  //sensor is on, want 1x oversampling on temps, 16x oversample on pressure, no filter, minimal standby
  //rms noise: 0.005[C], 1.3[Pa]
  external_barometer.setSampling(external_barometer.MODE_NORMAL, external_barometer.SAMPLING_X1, external_barometer.SAMPLING_X16, external_barometer.FILTER_OFF, external_barometer.STANDBY_MS_1);


  
  //setup SDcard reader
  Serial.print("Initializing SD card...");
  if (!SD.begin(10)) {
    Serial.println("initialization failed!");
    while (1) {
      digitalWrite(RED, HIGH);
      delay(1000);
      digitalWrite(RED, LOW);
      delay(1000);
    }
  }
  Serial.println("initialization done.");

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
    //flightData.println("Time[ms], Temp[C], Pressure[Pa]");
    flightData.print("Time[ms], Temp(baro)[C], Pressure[Pa], Quat[e0], Quat[e1], Quat[e2], Quat[e3],");
    flightData.print(" Omega1[rad/s], Omega2[rad/s], Omega3[rad/s], acc1[m/s2], acc2[m/s2], acc3[m/s2],");
    flightData.print(" mag1[uT], mag2[uT], mag3[uT],");
    flightData.print(" lin acc1[m/s2], lin acc2[m/s2], lin acc3[m/s2],");
    flightData.print(" g1[m/s2], g2[m/s2], g3[m/s2],");
    //Onboard Sensors Start
    flightData.print(" Temp[F], Pressure[bar], gyro1[dps], gyro2[dps], gyro3[dps],");
    flightData.println(" acc1[g], acc2[g], acc3[g]");
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
  
  
  t_prev = millis();
  Serial.println("Startup done...");
  digitalWrite(GREEN,HIGH);
  digitalWrite(BLUE,HIGH);
  digitalWrite(RED,HIGH); //all lights for go
}


void loop() {
  if (sixteenIMU.accelerationAvailable()) {
    sixteenIMU.readAcceleration(acc_x, acc_y, acc_z);
  }
  if (sixteenIMU.gyroscopeAvailable()) {
    sixteenIMU.readGyroscope(gyro_x, gyro_y, gyro_z);
  }
  temp = HS300x.readTemperature();
  pressure = BARO.readPressure();

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
  
}
