
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/* This driver reads raw data from the BNO055

   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3.3V DC
   Connect GROUND to common ground

   History
   =======
   2015/MAR/03  - First release (KTOWN)
*/

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire);

float gyro_x_mod , gyro_y_mod , gyro_z_mod  , gyro_t_mod; 
float array[10],sum=0;


/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup(void)
{
  Serial.begin(115200);

  while (!Serial) delay(10);  // wait for serial port to open!

  Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);

  /* Display the current temperature */
  int8_t temp = bno.getTemp();
  Serial.print("Current Temperature: ");
  Serial.print(temp);
  Serial.println(" C");
  Serial.println("");

  bno.setExtCrystalUse(true);

  Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");
  Serial.println("<<<<<<<< Starting >>>>>>>");  
   delay(4000);
}

/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/
void loop(void)
{
  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  
  // uint8_t x1,x2; 
  /* Display the floating point data */
  Serial.print("EulerX: ");
  Serial.print(euler.x());
  Serial.print("EulerY: ");
  Serial.print(euler.y());
  Serial.print("EulerZ: ");
  Serial.print(euler.z());
  Serial.println("\t\t");
  // x1 = euler.x();
   
  // x2 = euler.x();
    
  imu::Vector<3> accele = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  
  
  Serial.print("AccX: ");
  Serial.print(accele.x());
  Serial.print("AccY: ");
  Serial.print(accele.y());
  Serial.print("AccZ: ");
  Serial.print(accele.z());
  Serial.print("\t\t");
 

  // imu::Vector<3> gyros = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);


  



   
  
  for(int i =0; i < sizeof(array); i++){
  imu::Vector<3> gyros = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);    

  gyro_x_mod = sqrt(gyros.x() * gyros.x());
  gyro_y_mod = sqrt(gyros.y() * gyros.y());
  gyro_z_mod = sqrt(gyros.z() * gyros.z()); 


  gyro_t_mod = gyro_x_mod + gyro_y_mod + gyro_z_mod ;     

  array[i]= gyro_t_mod;
  sum = sum + array[i];


  }
   
  sum = sum / sizeof(array);

  if( sum < 24 ){       
    Serial.println("Deflate");
     
  }else{
  Serial.println("Inflate");
  }

  //Setting caliberation 
  
      

  // // Quaternion data
  // imu::Quaternion quat = bno.getQuat();
  // Serial.print("qW: ");
  // Serial.print(quat.w(), 4);
  // Serial.print(" qX: ");
  // Serial.print(quat.x(), 4);
  // Serial.print(" qY: ");
  // Serial.print(quat.y(), 4);
  // Serial.print(" qZ: ");
  // Serial.print(quat.z(), 4);
  // Serial.print("\t\t");

  /* Display calibration status for each sensor. */
  // uint8_t system, gyro, accel, mag = 0;
  // bno.getCalibration(&system, &gyro, &accel, &mag);
  // Serial.print("CALIBRATION: Sys=");
  // Serial.print(system, DEC);
  // Serial.print(" Gyro=");
  // Serial.print(gyro, DEC);
  // Serial.print(" Accel=");
  // Serial.print(accel, DEC);
  // Serial.print(" Mag=");
  // Serial.println(mag, DEC);
  // Serial.print("\t\t");


  delay(BNO055_SAMPLERATE_DELAY_MS);
}
