#include <Wire.h>
#include <Adafruit_Sensor.h>    ////
#include <Adafruit_ICM20649.h>     ///

///
Adafruit_ICM20649 icm;         

// Constant for converting radians to degrees
#define RAD_TO_DEG 57.295779513F 

//Your original global variables 
float RateRoll, RatePitch, RateYaw;
float AngleRoll, AnglePitch;
float AccX, AccY, AccZ;
float AccZInertial;
float LoopTimer;

//// BMP280 variables -
uint16_t dig_T1, dig_P1;
int16_t  dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5;
int16_t  dig_P6, dig_P7, dig_P8, dig_P9;
float AltitudeBarometer, AltitudeBarometerStartUp;
int RateCalibrationNumber;

/// BasicLinearAlgebra 
#include <BasicLinearAlgebra.h>
using namespace BLA;
float AltitudeKalman, VelocityVerticalKalman;
BLA::Matrix<2, 2> F; BLA::Matrix<2, 1> G;
BLA::Matrix<2, 2> P; BLA::Matrix<2, 2> Q;
BLA::Matrix<2, 1> S; BLA::Matrix<1, 2> H;
BLA::Matrix<2, 2> I; BLA::Matrix<1, 1> Acc;
BLA::Matrix<2, 1> K; BLA::Matrix<1, 1> R;
BLA::Matrix<1, 1> L; BLA::Matrix<1, 1> M;

///// 2D Kalman filter 
void kalman_2d(void) {
  Acc = {AccZInertial};
  S = F * S + G * Acc;
  P = F * P * ~F + Q;
  L = H * P * ~H + R;
  K = P * ~H * Invert(L);
  M = {AltitudeBarometer};
  S = S + K * (M - H * S);
  AltitudeKalman = S(0, 0);
  VelocityVerticalKalman = S(1, 0);
  P = (I - K * H) * P;
}

///////// barometer function
void barometer_signals(void) {
  Wire.beginTransmission(0x76);
  Wire.write(0xF7);
  Wire.endTransmission();
  Wire.requestFrom(0x76, 6);
  uint32_t press_msb = Wire.read();
  uint32_t press_lsb = Wire.read();
  uint32_t press_xlsb = Wire.read();
  uint32_t temp_msb = Wire.read();
  uint32_t temp_lsb = Wire.read();
  uint32_t temp_xlsb = Wire.read();
  unsigned long int adc_P = (press_msb << 12) | (press_lsb << 4) | (press_xlsb >> 4);
  unsigned long int adc_T = (temp_msb << 12) | (temp_lsb << 4) | (temp_xlsb >> 4);
  signed long int var1, var2;
  var1 = ((((adc_T >> 3) - ((signed long int )dig_T1 << 1))) * ((signed long int )dig_T2)) >> 11;
  var2 = (((((adc_T >> 4) - ((signed long int )dig_T1)) * ((adc_T >> 4) - ((signed long int )dig_T1))) >> 12) * ((signed long int )dig_T3)) >> 14;
  signed long int t_fine = var1 + var2;
  unsigned long int p;
  var1 = (((signed long int )t_fine) >> 1) - (signed long int )64000;
  var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((signed long int )dig_P6);
  var2 = var2 + ((var1 * ((signed long int )dig_P5)) << 1);
  var2 = (var2 >> 2) + (((signed long int )dig_P4) << 16);
  var1 = (((dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + ((((signed long int )dig_P2) * var1) >> 1)) >> 18;
  var1 = ((((32768 + var1)) * ((signed long int )dig_P1)) >> 15);
  if (var1 == 0) {
    p = 0;
  }
  p = (((unsigned long int )(((signed long int ) 1048576) - adc_P) - (var2 >> 12))) * 3125;
  if (p < 0x80000000) {
    p = (p << 1) / ((unsigned long int ) var1);
  }
  else {
    p = (p / (unsigned long int )var1) * 2;
  }
  var1 = (((signed long int )dig_P9) * ((signed long int ) (((p >> 3) * (p >> 3)) >> 13))) >> 12;
  var2 = (((signed long int )(p >> 2)) * ((signed long int )dig_P8)) >> 13;
  p = (unsigned long int)((signed long int )p + ((var1 + var2 + dig_P7) >> 4));
  double pressure = (double)p / 100;
  AltitudeBarometer = 44330 * (1 - pow(pressure / 1013.25, 1 / 5.255)) * 100;
}


//// gyro_signals() 
void gyro_signals(void) {
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;

  // Get new sensor data from the library
  icm.getEvent(&accel, &gyro, &temp);

  ///  Gyroscope Data
  //// Convert from rad/s to degrees/s 
  RateRoll = gyro.gyro.x * RAD_TO_DEG;
  RatePitch = gyro.gyro.y * RAD_TO_DEG;
  RateYaw = gyro.gyro.z * RAD_TO_DEG;

  ///// Accelerometer Data
  /// Convert from m/s^2 (library) to g's (what your code expects)
  AccX = accel.acceleration.x / SENSORS_GRAVITY_STANDARD;
  AccY = accel.acceleration.y / SENSORS_GRAVITY_STANDARD;
  AccZ = accel.acceleration.z / SENSORS_GRAVITY_STANDARD;

  //// Angle Calculations (from your original code)
  /// Calculate angles from accelerometer data (in degrees)
  /// Using PI constant for better precision than 3.142
  AngleRoll = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * (180.0 / PI);
  AnglePitch = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * (180.0 / PI);
}



void setup() {
  Serial.begin(57600);  //// Baurate
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  Wire.setClock(400000);//// Speed of clk line forr I2C
  Wire.begin();
  delay(250);

  if (!icm.begin_I2C()) {
    Serial.println("Failed to find ICM20649 chip. Freezing.");
    while (1) {
      delay(10);
    }
  }
  Serial.println("ICM20649 Found!");

  
  ///// +/- 2000 dps
  /// +/- 16 g
  icm.setAccelRange(ICM20649_ACCEL_RANGE_16_G);
  icm.setGyroRange(ICM20649_GYRO_RANGE_2000_DPS);
  
  icm.setAccelRateDivisor(3);  // 1125 / (1+3) = 281.25 Hz
  icm.setGyroRateDivisor(3);   // 1100 / (1+3) = 275 Hz


  uint16_t a_div = icm.getAccelRateDivisor(); 
  uint16_t  g_div = icm.getGyroRateDivisor();

  float accel_rate = 1125.0 / (1.0 + a_div);
  float gyro_rate  = 1100.0 / (1.0 + g_div);

  //////// BMP280 initialization 
  Wire.beginTransmission(0x76);
  Wire.write(0xF4);   /// Register CTRL_MEAS
  Wire.write(0x57);   
  Wire.endTransmission();
  Wire.beginTransmission(0x76);
  Wire.write(0xF5);     //// Register CONFIG
  Wire.write(0x14);
  Wire.endTransmission();

  uint8_t data[24], i = 0;
  Wire.beginTransmission(0x76);
  Wire.write(0x88);
  Wire.endTransmission();
  Wire.requestFrom(0x76, 24);
  while (Wire.available()) {
    data[i] = Wire.read();
    i++;
  }
  dig_T1 = (data[1] << 8) | data[0];
  dig_T2 = (data[3] << 8) | data[2];
  dig_T3 = (data[5] << 8) | data[4];
  dig_P1 = (data[7] << 8) | data[6];
  dig_P2 = (data[9] << 8) | data[8];
  dig_P3 = (data[11] << 8) | data[10];
  dig_P4 = (data[13] << 8) | data[12];
  dig_P5 = (data[15] << 8) | data[14];
  dig_P6 = (data[17] << 8) | data[16];
  dig_P7 = (data[19] << 8) | data[18];
  dig_P8 = (data[21] << 8) | data[20];
  dig_P9 = (data[23] << 8) | data[22];
  delay(250);

  /////Barometer calibration 
  for (RateCalibrationNumber = 0; RateCalibrationNumber < 2000; RateCalibrationNumber++) {
    barometer_signals();
    AltitudeBarometerStartUp += AltitudeBarometer;
    delay(1);
  }
  AltitudeBarometerStartUp /= 2000;


  //Kalman filter set up
  F = {1, 0.004,
       0, 1
      };
  G = {0.5 * 0.004 * 0.004,
       0.004
      };
  H = {1, 0};
  I = {1, 0,
       0, 1
      };
  Q = G * ~G * 10 * 10;
  R = {30 * 30};
  P = {0, 0,
       0, 0
      };
  S = {0,
       0
      };
  LoopTimer = micros();
}




void loop() {
  gyro_signals(); /// ICM tx rx

  AccZInertial = -sin(AnglePitch * (PI / 180.0)) * AccX
                 + cos(AnglePitch * (PI / 180.0)) * sin(AngleRoll * (PI / 180.0)) * AccY
                 + cos(AnglePitch * (PI / 180.0)) * cos(AngleRoll * (PI / 180.0)) * AccZ;

  AccZInertial = (AccZInertial - 1) * 9.81 * 100;

  barometer_signals(); /// start tx rx barometer

  AltitudeBarometer -= AltitudeBarometerStartUp;  // subtract biases from calibration

  kalman_2d();   /// filter

  Serial.print(AltitudeKalman);
  Serial.print(" , ");
  Serial.print(VelocityVerticalKalman);
  Serial.print(" , ");
  Serial.println();
  while (micros() - LoopTimer < 4000); // Unchanged
  LoopTimer = micros();
}
