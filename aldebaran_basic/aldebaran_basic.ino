#include <math.h>
#include <Arduino.h>
#include "BasicStepperDriver.h"
//external cards
#include <SD.h>
#include <Adafruit_BNO055.h>  //ext. imu
#include <Adafruit_BMP280.h>  //ext pressure/temp

#define MOTOR_STEPS 200  //steps per srev
#define RPM 150          //speed? fix this later - marbe
#define MICROSTEPS 8
#define STEP 8
#define DIRECTION 9

#define HITL

#define DTF

//jimmy rigged hardware in loop testing arrays - ctrl F rustled jimmy to skip
float tableInterpHeight(float time){
  float height[] = {304.495200000000,
312.568225162652,
320.596321854462,
328.579303892293,
336.516837000289,
344.408502140038,
352.253828516105,
360.052318297067,
367.803456921657,
375.506720119497,
383.161578459696,
390.767499873299,
398.323958742942,
405.830440619147,
413.286449579955,
420.691507531359,
428.045153267467,
435.346943855123,
442.596455763612,
449.793285753067,
456.937049192816,
464.027377872760,
471.063925101771,
478.046368278973,
484.974412785377,
491.847787648299,
498.666243208132,
505.429659860666,
512.138497122598,
518.793587860165,
525.395453922713,
531.944645117282,
538.441954253931,
544.887863662755,
551.282891631837,
557.627802316399,
563.923048628167,
570.169105971693,
576.366671418852,
582.516157682781,
588.618011251827,
594.672878660809,
600.681143839823,
606.643223220176,
612.559733282613,
618.431031289234,
624.257501844130,
630.039702467961,
635.777962867561,
641.472647233773,
647.124282286781,
652.733173035229,
658.299656477871,
663.824225801536,
669.307165555204,
674.748793633739,
680.149567961027,
685.509755198651,
690.829654486752,
696.109696055943,
701.350126448420,
706.551222645611,
711.713379852310,
716.836827094150,
721.921823975658,
726.968734576925,
731.977772188347,
736.949183168389,
741.883324201842,
746.780400604074,
751.640644354120,
756.464375189983,
761.251781495392,
766.003079743307,
770.718570708474,
775.398430869051,
780.042863466293,
784.652145534397,
789.226442420917,
793.765946378063,
798.270920485314,
802.741521383111,
807.177932730877,
811.580406316281,
815.949091775014,
820.284160433904,
824.585836728714,
828.854258821238,
833.089588664904,
837.292045104769,
841.461759622861,
845.598884419397,
849.703621159685,
853.776093730399,
857.816445209537,
861.824861436706,
865.801459431575,
869.746376358525,
873.659795045497,
877.541827138577,
881.392600244801,
885.212277425584,
889.000963820513,
892.758780785275,
896.485883839090,
900.182373016498,
903.848363716615,
907.484004477959,
911.089391486011,
914.664635611860,
918.209879402238,
921.725214465121,
925.210744866532,
928.666601643123,
932.092870644080,
935.489648547987,
938.857055905957,
942.195174325922,
945.504095919245,
948.783935916434,
952.034772150804,
955.256692355042,
958.449806664134,
961.614189292745,
964.749923778680,
967.857115364864,
970.935834800127,
973.986161596107,
977.008196306943,
980.002006354174,
982.967666364100,
985.905269428606,
988.814879723035,
991.696567228714,
994.550418312859,
997.376494145619,
1000.17486144388,
1002.94560293035,
1005.68877699402,
1008.40444816435,
1011.09269801021,
1013.75358225740,
1016.38716036380,
1018.99350593403,
1021.57267201662,
1024.12471621557,
1026.64971108937,
1029.14770724582,
1031.61875945976,
1034.06293713843,
1036.48028846714,
1038.87086467772,
1041.23473039524,
1043.57193147336,
1045.88251662432,
1048.16654772460,
1050.42406841930,
1052.65512497923,
1054.85977662038,
1057.03806484374,
1059.19003287693,
1061.31573596759,
1063.41521357496,
1065.48850736310,
1067.53567153338,
1069.55674353086,
1071.55176221489,
1073.52077817329,
1075.46382691964,
1077.38094524507,
1079.27218153382,
1081.13756942607,
1082.97714369863,
1084.79095059274,
1086.57902191987,
1088.34139003640,
1090.07809817007,
1091.78917640752,
1093.47465569174,
1095.13457819216,
1096.76897222752,
1098.37786649873,
1099.96130041777,
1101.51930065250,
1103.05189448934,
1104.55912022745,
1106.04100282939,
1107.49756734229,
1108.92884924283,
1110.33487163467,
1111.71565778791,
1113.07124149611,
1114.40164434601,
1115.70688803824,
1116.98700481165,
1118.24201486874,
1119.47193823592,
1120.67680505596,
1121.85663409428,
1123.01144387152,
1124.14126301247,
1125.24610890006,
1126.32599879092,
1127.38096021747,
1128.41100903145,
1129.41616082499,
1130.39644119368,
1131.35186457605,
1132.28244517916,
1133.18820739495,
1134.06916415457,
1134.92532808785,
1135.75672177063,
1136.56335675026,
1137.34524425644,
1138.10240546288,
1138.83485055370,
1139.54258931915,
1140.22564143469,
1140.88401563413,
1141.51772024787,
1142.12677352774,
1142.71118284359,
1143.27095510839,
1143.80610710427,
1144.31664469278,
1144.80257668950,
1145.26391142185,
1145.70065679529,
1146.11281683412,
1146.50040532455,
1146.86342520123,
1147.20187889203,
1147.51577858651,
1147.80512566050,
1148.06992443733,
1148.31017878356,
1148.52589207561,
1148.71706381614,
1148.88370337911,
1149.02580953951,
1149.14338424705,
1149.23642927036,
1149.30494622289,
1149.34893309011,
1149.36839777139};
  for(int i=0; i<246; i++)
  {
    if (0.05*i > time) return height[i];
  }
  return -1.0;
}
float tableInterpAccel(float time){
  float accel[] = {-17.9899546290145,
-17.9627799713000,
-18.0412687548629,
-18.1768200165985,
-18.3454478965729,
-18.5347055497405,
-18.7339764872018,
-18.9401078930110,
-19.1500426881980,
-19.3617129653766,
-19.5749708704820,
-19.7853285349587,
-19.9914570050769,
-20.1896389772651,
-20.3809272831977,
-20.5654570658664,
-20.7426687630611,
-20.9121119739218,
-21.0734274854284,
-21.2270302726156,
-21.3746110295308,
-21.5132981734863,
-21.6428049839925,
-21.7601877092696,
-21.8685776963371,
-21.9679789229210,
-22.0614116833883,
-21.8679849271989,
-21.4803109426704,
-21.2896007073667,
-21.1019636365257,
-20.7380459369924,
-20.5596528192232,
-20.3832708675018,
-20.0308127676377,
-19.8654229103930,
-19.7045736822966,
-19.3833515379675,
-19.2317607162523,
-19.0803837918605,
-18.7840870220397,
-18.6406915260687,
-18.5004848575140,
-18.2156623987632,
-18.0846596302465,
-17.9554763142945,
-17.7000040856973,
-17.5759346481319,
-17.4536377967698,
-17.2120342713248,
-17.0975840465141,
-16.9850164176466,
-16.7585255156352,
-16.6518446004714,
-16.5453299869048,
-16.3362843703933,
-16.2347173924626,
-16.1347415227516,
-15.9381769245063,
-15.8443664449607,
-15.7519670692125,
-15.5718046309534,
-15.4838893284979,
-15.3970694232140,
-15.2320241454840,
-15.1491066963433,
-15.0673118987660,
-14.9046577059216,
-14.8257660341843,
-14.7478532503518,
-14.6035976392328,
-14.5297154794739,
-14.4572509461235,
-14.3215164154014,
-14.2522500843866,
-14.1838327816149,
-14.0595744777316,
-13.9940054082154,
-13.9292953008307,
-13.8114058073339,
-13.7493474825053,
-13.6872940868709,
-13.5746561896979,
-13.5151892209378,
-13.4565069799344,
-13.3572846740237,
-13.3016256018334,
-13.2466772218892,
-13.1494453785654,
-13.0967167585149,
-13.0446040338098,
-12.9557519518418,
-12.9056187184367,
-12.8561167632199,
-12.7749074165274,
-12.7272461497632,
-12.6801511050839,
-12.5999122809851,
-12.5545936298967,
-12.5098399535182,
-12.4394358424331,
-12.3962724420461,
-12.3536237634857,
-12.2866469906470,
-12.2455107168064,
-12.2048637097769,
-12.1411491240782,
-12.1014618049810,
-12.0622593671335,
-12.0012478403160,
-11.9634534036601,
-11.9261203263272,
-11.8706926835157,
-11.8350758636700,
-11.7998779435522,
-11.7495954912397,
-11.7155421735084,
-11.6819041861757,
-11.6340160739533,
-11.6014735164228,
-11.5693289781600,
-11.5237373391301,
-11.4926418065206,
-11.4619280577460,
-11.4185398056651,
-11.3888312343358,
-11.3594891480574,
-11.3182157094357,
-11.2898373864292,
-11.2618111875822,
-11.2242094067405,
-11.1970808642419,
-11.1702828455230,
-11.1360417302458,
-11.1100743734507,
-11.0844247446412,
-11.0517929418275,
-11.0269438131464,
-11.0024019392626,
-10.9699554543773,
-10.9462149494759,
-10.9227727494947,
-10.8945008125152,
-10.8717713724512,
-10.8493314119098,
-10.8211850007771,
-10.7994639751945,
-10.7780221689365,
-10.7512650598050,
-10.7305177149977,
-10.7100412594571,
-10.6856660849268,
-10.6658341047094,
-10.6462668006238,
-10.6230838849417,
-10.6041411572387,
-10.5854559603298,
-10.5634261203874,
-10.5453465874379,
-10.5275178262963,
-10.5074395578998,
-10.4901732371225,
-10.4731549103676,
-10.4532965265986,
-10.4368495694037,
-10.4206412987781,
-10.4025414795108,
-10.3868656638338,
-10.3714259199981,
-10.3542606750138,
-10.3393398630047,
-10.3246498098361,
-10.3083924183202,
-10.2942084429483,
-10.2802502071304,
-10.2654196847014,
-10.2519404142528,
-10.2386804273868,
-10.2241454271918,
-10.2113683623588,
-10.1988081342271,
-10.1855580706776,
-10.1734562957003,
-10.1615657651013,
-10.1486631992200,
-10.1372347351918,
-10.1260791132582,
-10.1144545490441,
-10.1037873506244,
-10.0933155770780,
-10.0823837920349,
-10.0723273671172,
-10.0624641399650,
-10.0521693718088,
-10.0426704199866,
-10.0333664868175,
-10.0240007105441,
-10.0150966547404,
-10.0063859007500,
-9.99763604453105,
-9.98928517981923,
-9.98112919815917,
-9.97277776833581,
-9.96502909848804,
-9.95747296997956,
-9.94994197287478,
-9.94277866108551,
-9.93580644540226,
-9.92874166969659,
-9.92216678749814,
-9.91578131299904,
-9.90946696649263,
-9.90346715749023,
-9.89763390030936,
-9.89188394368862,
-9.88643026875802,
-9.88116886078904,
-9.87602189924441,
-9.87115050181478,
-9.86647079208469,
-9.86189307535132,
-9.85756910212403,
-9.85344352760757,
-9.84947089185803,
-9.84574645834831,
-9.84222016873880,
-9.83889223982205,
-9.83572606530324,
-9.83275419925760,
-9.82997327044335,
-9.82742804386498,
-9.82509238920187,
-9.82295192290670,
-9.82103152886548,
-9.81930418372469,
-9.81775558045699,
-9.81640646902173,
-9.81523932124862,
-9.81422928139874,
-9.81335278555488,
-9.81257528194766,
-9.81186887247884,
-9.81122186254828,
-9.81065426398045,
-9.81022469932205,
-9.81001204076012};
  int index = 0;
  for(int i=0; i<246; i++)
  {
    if (0.05*i > time) return accel[i];
  }
  return -1.0;
}
//rustled jimmy
unsigned int currentStep = 0;
float angleCommand = 0.0;
//External Sensor variables
Adafruit_BMP280 external_barometer;
Adafruit_BNO055 external_IMU;
long ext_ID = 55;
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;

//Data Recording
File flightData;
String fileName;

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
float velocity = 0;

//K535 data
float groundPressure;
float minburn = 2.5;           //sec
float expected_max_acc = 8.9;  //G


float time_since_burnout = 0;
float predicted_apogee = 0;
float desired_apogee = 0;
float drag_flap_angle = 0;

//TMC2226 is a TMC2209 in a different package

BasicStepperDriver stepper(MOTOR_STEPS, DIRECTION, STEP);

#ifndef PI
#define PI 3.14159265358979323846
#endif

bool degMode = true;
float angleCommand_rad = 0.0;

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
 

void setup() {

  stepper.begin(RPM, MICROSTEPS);
  //Serial link initialization
  Serial.begin(9600);  //baud rate goes brrrrr

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
long unsigned burnout_time = 0;
float timeSinceBurnout=0;
float lastStep=0;
float timeAtBurnout=0;

bool launched = true;
bool burnout = false;
bool apogee = false;
bool startedup = false;
void loop() {
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

  acc_x = tableInterpAccel(t_now/ 1000000.0f);

  t_prev = t_now;
  t_now = micros();

  float dt = (t_now - t_prev) / 1000000.0f;
  velocity += accelerint(acc_x_t1, acc_x, dt);
  velocity = 160.0;
  float altitude = pressureAlt(pressure) - pressureAlt(groundPressure);
  altitude = tableInterpHeight(t_now/ 1000000.0f);
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
    
    acc_x = tableInterpAccel(t_now/ 1000000.0f);
    t_prev = t_now;
    t_now = micros();
    dt = (t_now - t_prev) / 1000000.0f;
    velocity += accelerint(acc_x_t1, acc_x, dt);
    altitude = pressureAlt(pressure) - pressureAlt(groundPressure);
    altitude = tableInterpHeight(t_now/ 1000000.0f);
    
    launched=(acc_x>=50);
    Serial.println("YEET YEET");
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
  float ang = getControl(getDesired(timeSinceBurnout), predictAltitude(altitude, velocity), timeSinceBurnout-lastStep);
  lastStep = timeSinceBurnout;
  Serial.print("Moving to angle ");
  Serial.println(ang);
  moveToAngle(ang);
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
  flightData.print(String(velocity) + ',');
  flightData.println(String(predicted_apogee) + ',' + String(desired_apogee) + ',' + String(drag_flap_angle) + ',' + String(currentStep) + '\n');
  flightData.flush();
#endif
}
