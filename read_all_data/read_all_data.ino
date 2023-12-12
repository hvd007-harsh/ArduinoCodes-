#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

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

/* Set the delay between fresh samples */
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;
double angle_x = -1000000, angle_y = -1000000 , angle_z = -1000000;
double gx = -1000000, gy = -1000000 , gz = -1000000;
double linear_x = -1000000, linear_y = -1000000 , linear_z = -1000000;
// double gcc_x = -1000000, gcc_y = -1000000 , gcc_z = -1000000;
float elapsedTime, currentTime, previousTime;
double x = -1000000, y = -1000000 , z = -1000000; 
double tracker_x = 0 , tracker_z =0 ; 

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

void setup(void)
{
  Serial.begin(115200);

  while (!Serial) delay(10);  // wait for serial port to open!

  Serial.println("Orientation Sensor Test"); Serial.println("");

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  previousTime = currentTime;        // Previous time is stored before the actual time read
  currentTime = millis();            // Current time actual time read
  elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds

  delay(1000);
}

void loop(void)
{
  //could add VECTOR_ACCELEROMETER, VECTOR_MAGNETOMETER,VECTOR_GRAVITY...
  sensors_event_t orientationData , angVelocityData , linearAccelData, magnetometerData, accelerometerData, gravityData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);

  printEvent(&orientationData);
  //Correcting the value 
  angle_z = angle_z +59;
  angle_y = angle_y - 8 ; 
  angle_x = angle_x + 1 ;


  Serial.print("\tangle_x= ");
  Serial.print(angle_x);
  Serial.print(" |\ty= ");
  Serial.print(angle_y);
  Serial.print(" |\tz= ");
  Serial.println(angle_z);


  printEvent(&angVelocityData);




  Serial.print("\t gx= ");
  Serial.print(gx);
  Serial.print(" |\t gy= ");
  Serial.print(gy);
  Serial.print(" |\t gz= ");
  Serial.println(gz);
  


  // if( gx > 10){
  //   tracker_x = gx ; 
  //   if(tracker_x > -5){
  //     tracker_x = tracker_x - gx ; 
  //   }
  // }
  
  // if( gx < -10){
  //   tracker_x = gx  ; 
  //   if(tracker_x > 5){
  //     tracker_x = tracker_x + gx ; 
  //   } 
  // }

  //   if( gz > 10){
  //   tracker_z = gz ; 
  //   if(tracker_z > -5){
  //     tracker_z = tracker_z - gx ; 
  //   }
  // }
  
  // if( gz < -10){
  //   tracker_z = gz  ; 
  //   if(tracker_z > 5){
  //     tracker_z = tracker_z + gx ; 
  //   } 
  // }
  




   
  // printEvent(&linearAccelData);
  // printEvent(&magnetometerData);
  printEvent(&accelerometerData);
  // Serial.print("\tx= ");
  // Serial.print(x);
  // Serial.print(" |\ty= ");
  // Serial.print(y);
  // Serial.print(" |\tz= ");
  // Serial.println(z);
  // printEvent(&gravityData);
  x = x+ 1.09 ; 
  y = y - 9.76; 
  z = z + 1.20; 

  Serial.print("\tx= ");
  Serial.print(x);
  Serial.print(" |\ty= ");
  Serial.print(y);
  Serial.print(" |\tz= ");
  Serial.println(z);
  // printEvent(&gravityData);





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

  Serial.println("--");
  delay(BNO055_SAMPLERATE_DELAY_MS);
}

void printEvent(sensors_event_t* event) {
//dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_ACCELEROMETER) {
    Serial.print("Accl:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_ORIENTATION) {
    Serial.print("Orient:");
    angle_x = event->orientation.x;
    angle_y = event->orientation.y;
    angle_z = event->orientation.z;
  }
  else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
    Serial.print("Mag:");
    x = event->magnetic.x;
    y = event->magnetic.y;
    z = event->magnetic.z;
  }
  else if (event->type == SENSOR_TYPE_GYROSCOPE) {
    Serial.print("Gyro:");
    gx = event->gyro.x;
    gy = event->gyro.y;
    gz = event->gyro.z;
    gx = gx +  gx * elapsedTime * 180 /PI ;
    gy = gy +  gy * elapsedTime * 180 /PI ;
    gz = gz +  gz * elapsedTime * 180 /PI ;

  }
  else if (event->type == SENSOR_TYPE_ROTATION_VECTOR) {
    Serial.print("Rot:");
    gx = event->gyro.x;
    gy = event->gyro.y;
    gz = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
    Serial.print("Linear:");
    linear_x = event->acceleration.x;
    linear_y = event->acceleration.y;
    linear_z = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_GRAVITY) {
    Serial.print("Gravity:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else {
    Serial.print("Unk:");
  }
}


