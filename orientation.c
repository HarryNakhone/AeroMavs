#include <Wire.h>
#include <Adafruit_Sensor.h>    ////
#include <Adafruit_ICM20649.h>     ///
#include <math.h>

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

// Debug/plot control flags
const bool VERBOSE = false;      // set true to see detailed logs
const bool PLOTTER_OUTPUT = true; // set true to emit labeled single-line values for Arduino Serial Plotter

// Tunable parameters
const float ACCZ_LPF_CUTOFF_HZ = 15.0f;   // Low-pass cutoff for AccZ_inertial
const float TILT_LPF_CUTOFF_HZ = 3.0f;    // Low-pass cutoff for roll/pitch used in rotation
const float ACCZ_ABS_CLAMP = 50.0f;       // Safety clamp for |AccZ_inertial| in m/s^2
const float BARO_OUTLIER_THRESH_M = 2.0f; // Reject baro jumps larger than this per-sample
const float ACCZ_BIAS_ADAPT_RATE = 0.001f; // Bias estimator IIR rate when stationary
const float STATIONARY_GYRO_DPS = 3.0f;    // Gyro magnitude threshold for stationary (deg/s)
const float STATIONARY_ACCZ_MS2 = 0.5f;    // AccZ magnitude threshold for stationary (m/s^2)
const float VIBRATION_GYRO_LOW = 5.0f;     // Start of vibration range (deg/s)
const float VIBRATION_GYRO_HIGH = 150.0f;  // Strong vibration (deg/s)

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

// Diagnostics
float Innovation = 0.0f; // baro - predicted altitude (pre-update)

// Filters state
float AngleRollLPF = 0.0f;
float AnglePitchLPF = 0.0f;
float AccZInertialLPF = 0.0f;
float AccZBias = 0.0f;

// Barometer smoothing
float AltitudeBarometerFiltered = 0.0f;
float AltitudeBarometerPrevAccepted = 0.0f;
float BaroMA[3] = {0.0f, 0.0f, 0.0f};
int BaroMAIdx = 0;
int BaroMACount = 0;

///// 2D Kalman filter 
void kalman_2d(void) {
  Acc = {AccZInertial};
  S = F * S + G * Acc;
  P = F * P * ~F + Q;
  L = H * P * ~H + R;
  K = P * ~H * Inverse(L);
  M = {AltitudeBarometer};
  // Compute innovation (pre-update) for diagnostics
  BLA::Matrix<1,1> innov = M - H * S;
  Innovation = innov(0,0);
  S = S + K * innov;
  AltitudeKalman = S(0, 0);
  VelocityVerticalKalman = S(1, 0);
  P = (I - K * H) * P;
}

///////// barometer function
void barometer_signals(void) {
  Wire.beginTransmission(0x76);
  Wire.write(0xF7);
  Wire.endTransmission();
  int count = Wire.requestFrom(0x76, 6);
  if (VERBOSE && count != 6) {
    Serial.print("[BMP280][WARN] Data bytes requested=6, received=");
    Serial.println(count);
  }
  uint32_t press_msb = Wire.read();
  uint32_t press_lsb = Wire.read();
  uint32_t press_xlsb = Wire.read();
  uint32_t temp_msb = Wire.read();
  uint32_t temp_lsb = Wire.read();
  uint32_t temp_xlsb = Wire.read();
  unsigned long int adc_P = (press_msb << 12) | (press_lsb << 4) | (press_xlsb >> 4);
  unsigned long int adc_T = (temp_msb << 12) | (temp_lsb << 4) | (temp_xlsb >> 4);
  if (VERBOSE) {
    Serial.print("[BMP280] adc_T="); Serial.print(adc_T);
    Serial.print(" adc_P="); Serial.println(adc_P);
  }
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
    if (VERBOSE) Serial.println("[BMP280][ERROR] var1 == 0; skipping pressure calculation to avoid div-by-zero");
    return;
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
  // Altitude now in meters (removed previous *100 to cm conversion)
  AltitudeBarometer = 44330 * (1 - pow(pressure / 1013.25, 1 / 5.255));
  if (VERBOSE) {
    Serial.print("[BMP280] Pressure(hPa)="); Serial.print(pressure);
    Serial.print(" Alt(m)="); Serial.println(AltitudeBarometer);
  }
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
  if (VERBOSE) {
    Serial.print("[ICM] Acc(g)=");
    Serial.print(AccX); Serial.print(",");
    Serial.print(AccY); Serial.print(",");
    Serial.println(AccZ);
  }

  //// Angle Calculations (from your original code)
  /// Calculate angles from accelerometer data (in degrees)
  /// Using PI constant for better precision than 3.142
  AngleRoll = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * (180.0 / PI);
  AnglePitch = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * (180.0 / PI);
  if (VERBOSE) {
    Serial.print("[ICM] Gyro(dps)=");
    Serial.print(RateRoll); Serial.print(",");
    Serial.print(RatePitch); Serial.print(",");
    Serial.println(RateYaw);
    Serial.print("[ICM] Angles(deg) roll=");
    Serial.print(AngleRoll); Serial.print(" pitch=");
    Serial.println(AnglePitch);
  }
}



void setup() {
  Serial.begin(57600);  //// Baurate
  while (!Serial && millis() < 2000) {
    ; // For native USB boards, wait briefly so first prints are valid
  }
  if (VERBOSE) Serial.println("[BOOT] Starting... Ensure Serial Monitor = 57600 baud");
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
  if (VERBOSE) Serial.println("ICM20649 Found!");

  
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
  if (VERBOSE) {
    Serial.print("[ICM] Accel ODR ~ "); Serial.print(accel_rate); Serial.println(" Hz");
    Serial.print("[ICM] Gyro  ODR ~ "); Serial.print(gyro_rate);  Serial.println(" Hz");
  }

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
  int requested = Wire.requestFrom(0x76, 24);
  if (VERBOSE && requested != 24) {
    Serial.print("[BMP280][WARN] Calibration bytes requested=24, received=");
    Serial.println(requested);
  }
  while (Wire.available() && i < 24) {
    data[i] = Wire.read();
    i++;
  }
  if (VERBOSE && i != 24) {
    Serial.print("[BMP280][ERROR] Incomplete calibration block read, bytes=");
    Serial.println(i);
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
  if (VERBOSE) Serial.println("[BMP280] Calibration loaded.");

  /////Barometer calibration 
  for (RateCalibrationNumber = 0; RateCalibrationNumber < 2000; RateCalibrationNumber++) {
    barometer_signals();
    AltitudeBarometerStartUp += AltitudeBarometer;
    delay(1);
  }
  AltitudeBarometerStartUp /= 2000;
  if (VERBOSE) {
    Serial.print("[BMP280] Altitude baseline (m): ");
    Serial.println(AltitudeBarometerStartUp);
  }


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
  // Use a moderate, consistent process noise (1.0 m/s^2 sigma)
  Q = (G * ~G) * (1.0f * 1.0f);
  // R was 30 cm -> now 0.3 m
  R = {0.3f * 0.3f};
  P = {0, 0,
       0, 0
      };
  S = {0,
       0
      };
  LoopTimer = micros();
  if (VERBOSE) Serial.println("[KF] Initialized (dt=4ms assumed).");
  // Initialize filter states
  AngleRollLPF = 0.0f;
  AnglePitchLPF = 0.0f;
  AccZInertialLPF = 0.0f;
  AltitudeBarometerFiltered = 0.0f;
  AltitudeBarometerPrevAccepted = 0.0f;
}




void loop() {
  unsigned long now = micros();
  float dt = (now - LoopTimer) / 1000000.0f;
  if (dt <= 0.0f || dt > 0.1f) {
    dt = 0.004f; // Fallback to 4ms if timing is off
  }
  // Tight clamp for numerical stability
  if (dt < 0.003f) dt = 0.004f;
  if (dt > 0.006f) dt = 0.004f;

  gyro_signals(); /// ICM tx rx

  // Gyro magnitude (deg/s) for adaptive behavior
  float gyroMag = sqrtf(RateRoll * RateRoll + RatePitch * RatePitch + RateYaw * RateYaw);
  // Map vibration level 0..1
  float vib01 = (gyroMag - VIBRATION_GYRO_LOW) / (VIBRATION_GYRO_HIGH - VIBRATION_GYRO_LOW);
  if (vib01 < 0.0f) vib01 = 0.0f; if (vib01 > 1.0f) vib01 = 1.0f;

  // Low-pass filter for tilt used in rotation
  float a_tilt = expf(-2.0f * PI * TILT_LPF_CUTOFF_HZ * dt);
  if (!isfinite(a_tilt) || a_tilt < 0.0f || a_tilt > 1.0f) a_tilt = 0.9f;
  AngleRollLPF = a_tilt * AngleRollLPF + (1.0f - a_tilt) * AngleRoll;
  AnglePitchLPF = a_tilt * AnglePitchLPF + (1.0f - a_tilt) * AnglePitch;

  AccZInertial = -sin(AnglePitchLPF * (PI / 180.0)) * AccX
                 + cos(AnglePitchLPF * (PI / 180.0)) * sin(AngleRollLPF * (PI / 180.0)) * AccY
                 + cos(AnglePitchLPF * (PI / 180.0)) * cos(AngleRollLPF * (PI / 180.0)) * AccZ;

  // AccZInertial now in m/s^2 (no cm scaling)
  AccZInertial = (AccZInertial - 1) * 9.81;
  // Clamp extreme spikes and low-pass AccZ
  if (AccZInertial > ACCZ_ABS_CLAMP) AccZInertial = ACCZ_ABS_CLAMP;
  if (AccZInertial < -ACCZ_ABS_CLAMP) AccZInertial = -ACCZ_ABS_CLAMP;
  // Adaptive AccZ low-pass cutoff with vibration (more vibration => less filtering for responsiveness)
  float cutoffAcc = ACCZ_LPF_CUTOFF_HZ * (1.0f + 3.0f * vib01); // up to 4x cutoff at high vibration
  float a_acc = expf(-2.0f * PI * cutoffAcc * dt);
  if (!isfinite(a_acc) || a_acc < 0.0f || a_acc > 1.0f) a_acc = 0.9f;
  AccZInertialLPF = a_acc * AccZInertialLPF + (1.0f - a_acc) * AccZInertial;
  AccZInertial = AccZInertialLPF;

  // Stationary detector and bias adaptation for AccZ
  bool isStationary = (gyroMag < STATIONARY_GYRO_DPS) && (fabsf(AccZInertial) < STATIONARY_ACCZ_MS2);
  if (isStationary) {
    AccZBias = (1.0f - ACCZ_BIAS_ADAPT_RATE) * AccZBias + ACCZ_BIAS_ADAPT_RATE * AccZInertial;
  }
  AccZInertial -= AccZBias;
  if (VERBOSE) {
    Serial.print("[ICM] AccZ_inertial(m/s^2)=");
    Serial.println(AccZInertial);
  }

  barometer_signals(); /// start tx rx barometer

  AltitudeBarometer -= AltitudeBarometerStartUp;  // subtract biases from calibration
  // Baro outlier rejection and moving average smoothing (adaptive)
  float baroCandidate = AltitudeBarometer;
  if (BaroMACount > 0) {
    if (fabsf(baroCandidate - AltitudeBarometerPrevAccepted) > BARO_OUTLIER_THRESH_M) {
      baroCandidate = AltitudeBarometerPrevAccepted;
    } else {
      AltitudeBarometerPrevAccepted = baroCandidate;
    }
  } else {
    AltitudeBarometerPrevAccepted = baroCandidate;
  }
  int baroWindow = (vib01 > 0.6f) ? 1 : 3; // shorten window under vibration for responsiveness
  if (baroWindow == 1) {
    AltitudeBarometerFiltered = baroCandidate;
  } else {
    BaroMA[BaroMAIdx] = baroCandidate;
    BaroMAIdx = (BaroMAIdx + 1) % 3;
    if (BaroMACount < 3) BaroMACount++;
    float baroSum = 0.0f;
    for (int i = 0; i < BaroMACount; i++) baroSum += BaroMA[i];
    AltitudeBarometerFiltered = baroSum / (float)BaroMACount;
  }
  // Use filtered baro for KF
  AltitudeBarometer = AltitudeBarometerFiltered;
  if (VERBOSE) {
    Serial.print("[BMP280] Altitude detrended (m)=");
    Serial.println(AltitudeBarometer);
  }

  // Update Kalman matrices with current dt
  F = {1, dt,
       0, 1
      };
  G = {0.5f * dt * dt,
       dt
      };
  // Adaptive Q/R with vibration: more vibration => trust process more (smaller Q), trust baro less (larger R)
  float sigma_acc = 1.0f - 0.5f * vib01; // 1.0 .. 0.5
  if (sigma_acc < 0.3f) sigma_acc = 0.3f;
  Q = (G * ~G) * (sigma_acc * sigma_acc);
  float r_sigma = 0.3f + 0.5f * vib01; // 0.30 .. 0.80 m
  R = {r_sigma * r_sigma};

  kalman_2d();   /// filter

  if (PLOTTER_OUTPUT) {
    // Labeled single-line output for Arduino Serial Plotter
    Serial.print("Alt:"); Serial.print(AltitudeKalman, 3); // meters
    Serial.print('\t');
    Serial.print("Vel:"); Serial.print(VelocityVerticalKalman, 3); // m/s
    Serial.print('\t');
    Serial.print("AccZ:"); Serial.print(AccZInertial, 3); // m/s^2
    Serial.print('\t');
    Serial.print("Baro:"); Serial.print(AltitudeBarometer, 3); // meters
    Serial.print('\t');
    Serial.print("Innov:"); Serial.print(Innovation, 3);
    Serial.print('\t');
    Serial.print("GyMag:"); Serial.print(gyroMag, 1);
    Serial.print('\t');
    Serial.print("GyX:"); Serial.print(RateRoll, 2); // deg/s
    Serial.print('\t');
    Serial.print("GyY:"); Serial.print(RatePitch, 2); // deg/s
    Serial.print('\t');
    Serial.print("GyZ:"); Serial.print(RateYaw, 2); // deg/s
    Serial.print('\t');
    Serial.print("dt:"); Serial.println(dt, 6);
  } else if (VERBOSE) {
    Serial.print("[KF] Alt(m)="); Serial.print(AltitudeKalman);
    Serial.print(" Vel(m/s)="); Serial.print(VelocityVerticalKalman);
    Serial.print(" dt(s)="); Serial.println(dt);
  }

  while (micros() - now < 4000) {
    // busy wait to enforce loop period ~4ms
  }
  LoopTimer = micros();
}
