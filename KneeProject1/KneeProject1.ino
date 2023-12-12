#include <Wire.h>
const int MPU_addr_1 = 0x68;                             // I2C address of the first MPU-6050
const int MPU_addr_2 = 0x69;                             // I2C address of the second MPU-6050
int16_t AcX1, AcY1, AcZ1, Tmp1, AcX2, AcY2, AcZ2, Tmp2;  // definition of variables
int32_t anglex1, angley1, anglez1;
int32_t angle=0;
int32_t anglex2, angley2, anglez2;
int32_t steadyanglex, steadyangley, steadyanglez;
int32_t steadyanglex2 , steadyangley2, steadyanglez2;
int32_t DOF;
int minVal = 265;
int maxVal = 402;


void setup() {
  Wire.begin();
  Wire.beginTransmission(MPU_addr_1);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.begin(9600);
  Wire.begin();
  Wire.beginTransmission(MPU_addr_2);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.begin(9600);

  Wire.beginTransmission(MPU_addr_1);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr_1, 14, true);       // request a total of 14 registers
  int AcX1 = Wire.read() << 8 | Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  int AcY1 = Wire.read() << 8 | Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  int AcZ1 = Wire.read() << 8 | Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  // Tmp1 = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)

  // Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  // Wire.endTransmission(false);
  // Wire.requestFrom(MPU_addr_1, 14, true);       // request a total of 14 registers
  // int AcX1 = Wire.read() << 8 | Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  // int AcY1 = Wire.read() << 8 | Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  // int AcZ1 = Wire.read() << 8 | Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)

  int xAng = map(AcX1, minVal, maxVal, -90, 90);
  int yAng = map(AcY1, minVal, maxVal, -90, 90);
  int zAng = map(AcZ1, minVal, maxVal, -90, 90);
  anglex1 = RAD_TO_DEG * (atan2(-yAng, -zAng) + PI);
  angley1 = RAD_TO_DEG * (atan2(-xAng, -zAng) + PI);
  anglez1 = RAD_TO_DEG * (atan2(-yAng, -xAng) + PI);

  steadyanglex = anglex1;
  steadyangley = angley1;
  steadyanglez = anglez1;

  Wire.beginTransmission(MPU_addr_2);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr_2, 14, true);       // request a total of 14 registers
  int AcX2 = Wire.read() << 8 | Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  int AcY2 = Wire.read() << 8 | Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  int AcZ2 = Wire.read() << 8 | Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
                                                // Tmp2 = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)

  xAng = map(AcX2, minVal, maxVal, -90, 90);
  yAng = map(AcY2, minVal, maxVal, -90, 90);
  zAng = map(AcZ2, minVal, maxVal, -90, 90);

  anglex2 = RAD_TO_DEG * (atan2(-yAng, -zAng) + PI);
  angley2 = RAD_TO_DEG * (atan2(-xAng, -zAng) + PI);
  anglez2 = RAD_TO_DEG * (atan2(-yAng, -xAng) + PI);

  steadyanglex2 = anglex2 ; 
  steadyangley2 = angley2; 
  steadyanglez2 = anglez2;
}
float calculateKneeAngle(int AcY1 , int AcY2) {
  // Calculate the difference between the accelerometer readings
  float diffAccel = AcY2 - AcY1;

  // Convert the difference to degrees
  float kneeAngle = diffAccel * 180 / 9.8;

  // Return the knee angle
  return kneeAngle ; 
}

void loop() {
  Wire.beginTransmission(MPU_addr_1);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr_1, 14, true);       // request a total of 14 registers
  int AcX1 = Wire.read() << 8 | Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  int AcY1 = Wire.read() << 8 | Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  int AcZ1 = Wire.read() << 8 | Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  // Tmp1 = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  // Wire.write(0x68);  // starting with register 0x3B (ACCEL_XOUT_H)
  // Wire.endTransmission(false);
  // Wire.requestFrom(MPU_addr_1, 14, true);       // request a total of 14 registers
  // int gcX1 = Wire.read() << 8 | Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  // int gcY1 = Wire.read() << 8 | Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  // int gcZ1 = Wire.read() << 8 | Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  // // Tmp1 = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  
  int xAng = map(AcX1, minVal, maxVal, -90, 90);
  int yAng = map(AcY1, minVal, maxVal, -90, 90);
  int zAng = map(AcZ1, minVal, maxVal, -90, 90);
  anglex1 = RAD_TO_DEG * (atan2(-yAng, -zAng) + PI);
  angley1 = RAD_TO_DEG * (atan2(-xAng, -zAng) + PI);
  anglez1 = RAD_TO_DEG * (atan2(-yAng, -xAng) + PI);

  Wire.beginTransmission(MPU_addr_2);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr_2, 14, true);       // request a total of 14 registers
  int AcX2 = Wire.read() << 8 | Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  int AcY2 = Wire.read() << 8 | Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  int AcZ2 = Wire.read() << 8 | Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
                                                // Tmp2 = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)

  xAng = map(AcX2, minVal, maxVal, -90, 90);
  yAng = map(AcY2, minVal, maxVal, -90, 90);
  zAng = map(AcZ2, minVal, maxVal, -90, 90);

  anglex2 = RAD_TO_DEG * (atan2(-yAng, -zAng) + PI);
  angley2 = RAD_TO_DEG * (atan2(-xAng, -zAng) + PI);
  anglez2 = RAD_TO_DEG * (atan2(-yAng, -xAng) + PI);


  // // Serial.println(Tmp2 / 340.00 + 36.53);
  //  Serial.print((steadyanglex - anglex1));
  //  Serial.print("\t");
  // Serial.print( steadyangley - angley1);
  // Serial.print(" ");
  // Serial.print( steadyanglez - anglez1);
  // Serial.println(" ");

  // Serial.print( steadyanglex2 - anglex2);
  // Serial.print("\t");
  // Serial.print( steadyangley2 - angley2);
  // Serial.print(" ");
  // Serial.print( steadyanglez2 - anglez2);
  // Serial.println(" ");
  
  // DOF = 180 - (steadyanglex - anglex1) - ( steadyanglex2 - anglex2);

 DOF =  calculateKneeAngle(AcY1,AcY2);

 // DOF = (steadyangley - angley1) - ( steadyangley2 - angley2);
  
  Serial.print(DOF/16384);
  Serial.print("\t");
  Serial.print("Alignment\n");
  
  angle = DOF/16384 * 90 ;
  angle = angle /10;

 if( angle > 90 ){
    int temp=0 ;
    temp = angle-90;
    angle = angle + temp;
  }
 
  Serial.print(angle);
  Serial.print("\t");
  Serial.print("Angle\n");
  
  Serial.print(AcY1/1638.4);
  Serial.print("\t");
  Serial.print("Acceleration\n");
  Serial.print("\n");
  // Serial.print("Above angle ");
  // Serial.print(90 - angle );

  
  delay(300);

  // switch(DOF){
  //   case (DOF > 45) : 
  //     Serial.print("Very flexible");
  //     break;
  //   case (DOF > 65) : 
  //     Serial.print("Flexible");
  //     break;
  //   case (DOF > 75) : 
  //     Serial.print("Normal");
  //     break;
  //   default: 
  //     Serial.print("Less Flexible");
  //     break;
      
  // }

}