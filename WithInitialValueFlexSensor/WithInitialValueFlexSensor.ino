/*
Author : Harsh Dixit  & Prashant Chauhan 
Date: 28-04-2023

*/

/* This driver uses the Adafruit unified sensor library (Adafruit_Sensor),
   which provides a common 'type' for sensor data and some helper functions.

   To use this driver you will also need to download the Adafruit_Sensor
   library and include it in your libraries folder.

   You should also assign a unique ID to this sensor for use with
   the Adafruit Sensor API so that you can identify this particular
   sensor in any data logs, etc.  To assign a unique ID, simply
   provide an appropriate value in the constructor below (12345
   is used by default in this example).

   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3.3-5V DC
   Connect GROUND to common ground

   History
   =======
   2015/MAR/03  - First release (KTOWN)
*/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>



/* Set the delay between fresh samples */
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;
double x = -1000000, y = -1000000, z = -1000000;
double angle_x = -1000000, angle_y = -1000000, angle_z = -1000000;
double acc_x = -1000000, acc_y = -1000000, acc_z = -1000000;
double gcc_x = -1000000, gcc_y = -1000000, gcc_z = -1000000;
double gyroAngleX, gyroAngleY, gyroAngleZ;
double accAngleX, accAngleY;

double avg = 0, pitch, yaw, roll;
double pitch_initial, yaw_initial, roll_initial;
double sinitialangle_x, sinitialangle_y, sinitialangle_z;     // Sitting initial angle
double stinitialangle_x, stinitialangle_y, stinitialangle_z;  // standing initial angle
float elapsedTime, currentTime, previousTime, countTime;
bool tof = true, fall = true;
int sitting_count = 0, standing_count = 0, count = 0;
bool trigger = false;
bool decider;



// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

void setup(void) {


  uint32_t sensorValue = analogRead(A0);
  // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
  float voltage = sensorValue;
  voltage = voltage - 900;


  Serial.begin(115200);

  while (!Serial) delay(10);  // wait for serial port to open!

  Serial.println("Orientation Sensor Test");
  Serial.println("");

  /* Initialise the sensor */
  if (!bno.begin()) {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1)
      ;
  }

  delay(1000);

  sensors_event_t orientationData, angVelocityData, linearAccelData, magnetometerData, accelerometerData, gravityData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);

  accAngleX = (atan(acc_y / sqrt(pow(acc_x, 2) + pow(acc_z, 2))) * 180 / PI);
  accAngleY = (atan(-1 * acc_x / sqrt(pow(acc_y, 2) + pow(acc_z, 2))) * 180 / PI);

  previousTime = currentTime;                         // Previous time is stored before the actual time read
  currentTime = millis();                             // Current time actual time read
  elapsedTime = (currentTime - previousTime) / 1000;  // Divide by 1000 to get seconds

  printEvent(&orientationData);
  printEvent(&angVelocityData);
  // printEvent(&linearAccelData);
  // printEvent(&magnetometerData);
  printEvent(&accelerometerData);

  if (voltage > 65) {
    sinitialangle_x = angle_x;
    sinitialangle_y = angle_y;
    sinitialangle_z = angle_z;
    decider = true;
  }
  if (voltage < 65) {
    stinitialangle_x = angle_x;
    stinitialangle_y = angle_y;
    stinitialangle_z = angle_z;
    decider = false;
  }

  gyroAngleX = gcc_x * elapsedTime;  // deg/s * s = deg
  gyroAngleY = gcc_y * elapsedTime;
  yaw_initial = yaw + gcc_z * elapsedTime;


  // Complementary filter - combine acceleromter and gyro angle values
  roll_initial = gyroAngleX + accAngleX;
  pitch_initial = gyroAngleY + accAngleY;
}
void sitting() {

  uint32_t sensorValue = analogRead(A0);
  // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
  float voltage = sensorValue;
  voltage = voltage - 900;

  for (int i = 0; i < 100; i++) {


    sensors_event_t orientationData, angVelocityData, linearAccelData, magnetometerData, accelerometerData, gravityData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
    // bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
    // bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
    bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    // bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);

    accAngleX = (atan(acc_y / sqrt(pow(acc_x, 2) + pow(acc_z, 2))) * 180 / PI);
    accAngleY = (atan(-1 * acc_x / sqrt(pow(acc_y, 2) + pow(acc_z, 2))) * 180 / PI);

    previousTime = currentTime;                         // Previous time is stored before the actual time read
    currentTime = millis();                             // Current time actual time read
    elapsedTime = (currentTime - previousTime) / 1000;  // Divide by 1000 to get seconds

    printEvent(&orientationData);
    printEvent(&angVelocityData);
    // printEvent(&linearAccelData);
    // printEvent(&magnetometerData);
    printEvent(&accelerometerData);


    gyroAngleX = gcc_x * elapsedTime;  // deg/s * s = deg
    gyroAngleY = gcc_y * elapsedTime;
    yaw = yaw + gcc_z * elapsedTime;


    // Complementary filter - combine acceleromter and gyro angle values
    roll = gyroAngleX + accAngleX;
    pitch = gyroAngleY + accAngleY;




    // Print the values on the serial monitor
    // Serial.print("Roll:");
    // Serial.print(roll);
    // Serial.print("Pitch:");
    // Serial.print(pitch);
    // Serial.print("Yaw:");
    // Serial.println(yaw);



    //Checking the position
    // if(yaw-yaw_initial > 1.99){
    //   Serial.println("Standing");
    // }else{
    // Serial.println("Sitting");
    // }





    // if( yaw < -0.03 ){
    //   Serial.println("Standing");
    // }else{
    //   Serial.println("Sitting");
    // }


    if (pitch > 12) {
      tof = false;
    }

    if (yaw < 2) {
      tof = false;
    }
    // (angle_z  < 20) || (angle_z > 20)
    if (pitch < 12) {

      tof = true;
      if (((sinitialangle_z - angle_z) < 12)) {
        tof = true;
      } else if (((sinitialangle_x - angle_x) < -45) || ((sinitialangle_x - angle_x) < -110)) {
        tof = false;
      } else if (voltage < 55) {
        tof = false;
      } else if (voltage > 75) {
        tof = true;
      } else {
        tof = false;
      }
    }

    // Serial.print("\tx= ");
    // Serial.print((initialangle_x-angle_x));
    // Serial.print(" |\ty= ");
    // Serial.print((initialangle_y-angle_y));
    // Serial.print(" |\tz= ");
    // Serial.println((initialangle_z-angle_z));


    if ((gcc_x > 3) || (gcc_x < -3)) {
      fall = false;
    }
    if ((gcc_y > 3) || (gcc_y < -3)) {
      fall = false;
    }
    if ((gcc_z > 3) || (gcc_z < -3)) {
      fall = false;
    }
    if (tof == true) {
      count++;
    }
  }

  //For accuracy

  if (count > 50) {
    Serial.println("Sitting");
    Serial.println(sitting_count);
    countTime = currentTime - previousTime;
    sitting_count = sitting_count + countTime;
    // sitting_count = sitting_count / 10;
    standing_count = 0;
    count = 0;
  } else {
    Serial.println("Standing");
    Serial.println(standing_count);
    countTime = currentTime - previousTime;
    standing_count = standing_count + countTime;
    // standing_count = standing_count / 10;
    sitting_count = 0;
    count = 0;
  }


  /*
Method : 1 
   



*/


  /* Method :2 

  for(int i = 0 ; i < 20;i++){
 
    avg +=x; 
  }
  avg = avg/20;
  // printEvent(&gravityData);

  if( avg < -1.00){
    Serial.println("Standing Position");
  }else{
     Serial.println("Sitting Position");
  }
*/
  // int8_t boardTemp = bno.getTemp();
  // Serial.println();
  // Serial.print(F("temperature: "));
  // Serial.println(boardTemp);

  // uint8_t system, gyro, accel, mag = 0;
  // bno.getCalibration(&system, &gyro, &accel, &mag);
  // Serial.println();
  // Serial.print("Calibration: Sys=");
  // Serial.print(system);
  // Serial.print(" Gyro=");
  // Serial.print(gyro);
  // Serial.print(" Accel=");
  // Serial.print(accel);
  // Serial.print(" Mag=");
  // Serial.println(mag);

  // Serial.println("--");
}

void standing() {

  uint32_t sensorValue = analogRead(A0);
  // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
  float voltage = sensorValue;
  voltage = voltage - 900;


  for (int i = 0; i < 100; i++) {


    sensors_event_t orientationData, angVelocityData, linearAccelData, magnetometerData, accelerometerData, gravityData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
    // bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
    // bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
    bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    // bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);

    accAngleX = (atan(acc_y / sqrt(pow(acc_x, 2) + pow(acc_z, 2))) * 180 / PI);
    accAngleY = (atan(-1 * acc_x / sqrt(pow(acc_y, 2) + pow(acc_z, 2))) * 180 / PI);

    previousTime = currentTime;                         // Previous time is stored before the actual time read
    currentTime = millis();                             // Current time actual time read
    elapsedTime = (currentTime - previousTime) / 1000;  // Divide by 1000 to get seconds

    printEvent(&orientationData);
    printEvent(&angVelocityData);
    // printEvent(&linearAccelData);
    // printEvent(&magnetometerData);
    printEvent(&accelerometerData);


    gyroAngleX = gcc_x * elapsedTime;  // deg/s * s = deg
    gyroAngleY = gcc_y * elapsedTime;
    yaw = yaw + gcc_z * elapsedTime;


    // Complementary filter - combine acceleromter and gyro angle values
    roll = gyroAngleX + accAngleX;
    pitch = gyroAngleY + accAngleY;




    // Print the values on the serial monitor
    // Serial.print("Roll:");
    // Serial.print(roll);
    // Serial.print("Pitch:");
    // Serial.print(pitch);
    // Serial.print("Yaw:");
    // Serial.println(yaw);



    //Checking the position
    // if(yaw-yaw_initial > 1.99){
    //   Serial.println("Standing");
    // }else{
    // Serial.println("Sitting");
    // }





    // if( yaw < -0.03 ){
    //   Serial.println("Standing");
    // }else{
    //   Serial.println("Sitting");
    // }


    if (pitch > 12) {
      tof = false;
    }

    if (yaw < 2) {
      tof = false;
    }
    // (angle_z  < 20) || (angle_z > 20)
    if (pitch < 12) {

      tof = true;
      if (((stinitialangle_z - angle_z) < -22)) {
        tof = true;
      } else if (((stinitialangle_x - angle_x) < 0) || ((stinitialangle_x - angle_x) > 110)) {
        tof = false;
      } else if (voltage < 55) {
        tof = false;
      } else if (voltage > 75) {
        tof = true;
      } else {
        tof = false;
      }
    }




    if ((gcc_x > 3) || (gcc_x < -3)) {
      fall = false;
    }
    if ((gcc_y > 3) || (gcc_y < -3)) {
      fall = false;
    }
    if ((gcc_z > 3) || (gcc_z < -3)) {
      fall = false;
    }
    if (tof == true) {
      count++;
    }
  }

  //For accuracy

  if (count > 50) {
    Serial.println("Sitting");
    Serial.println(sitting_count);
    countTime = currentTime - previousTime;
    sitting_count = sitting_count + countTime;
    // sitting_count = sitting_count / 10;
    standing_count = 0;
    count = 0;
  } else {
    Serial.println("Standing");
    Serial.println(standing_count);
    countTime = currentTime - previousTime;
    Serial.print(countTime);
    standing_count = standing_count + countTime;
    // standing_count = standing_count / 10;
    sitting_count = 0;
    count = 0;
  }
}

void loop(void) {
  //could add VECTOR_ACCELEROMETER, VECTOR_MAGNETOMETER,VECTOR_GRAVITY...
  uint32_t sensorValue = analogRead(A0);
  // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
  float voltage = sensorValue;
  voltage = voltage - 900;
  if (decider == true) {
    sitting();
  } else {
    standing();
  }


  // Serial.print("\tx= ");
  // Serial.print((stinitialangle_x-angle_x));
  // Serial.print(" |\ty= ");
  // Serial.print((stinitialangle_y-angle_y));
  // Serial.print(" |\tz= ");
  // Serial.println((stinitialangle_z-angle_z));
}

void printEvent(sensors_event_t* event) {
  //dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_ACCELEROMETER) {
    // Serial.print("Accl:");
    acc_x = event->acceleration.x;
    acc_y = event->acceleration.y;
    acc_z = event->acceleration.z;
  } else if (event->type == SENSOR_TYPE_ORIENTATION) {
    // Serial.print("Orient:");
    angle_x = event->orientation.x;
    angle_y = event->orientation.y;
    angle_z = event->orientation.z;
  } else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
    Serial.print("Mag:");
    x = event->magnetic.x;
    y = event->magnetic.y;
    z = event->magnetic.z;
  } else if (event->type == SENSOR_TYPE_GYROSCOPE) {
    // Serial.print("Gyro:");
    gcc_x = event->gyro.x;
    gcc_y = event->gyro.y;
    gcc_z = event->gyro.z;
  } else if (event->type == SENSOR_TYPE_ROTATION_VECTOR) {
    Serial.print("Rot:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  } else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
    Serial.print("Linear:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  } else if (event->type == SENSOR_TYPE_GRAVITY) {
    Serial.print("Gravity:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  } else {
    Serial.print("Unk:");
  }

  // Serial.print("\tx= ");
  // Serial.print(x);
  // Serial.print(" |\ty= ");
  // Serial.print(y);
  // Serial.print(" |\tz= ");
  // Serial.println(z);
}
