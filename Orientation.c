// Basic demo for accelerometer readings from Adafruit ICM20649

#include <Adafruit_ICM20649.h>
#include <Adafruit_Sensor.h>
#include <MadgwickAHRS.h>
#include <Wire.h>


Adafruit_ICM20649 icm;

unsigned long microsPerReading, microsPrevious;


struct Calibrated_g {
  float x;
  float y;
  float z;
};


struct Calibrated_a{
  float x;
  float y;
  float z;
};

enum class SensorType{
  Gyro,
  Accel
};

Madgwick filter;

float gyro_biases [3] ={0};
float accel_biases [3] ={0};

struct Calibrated_g calibrated_gyro ={0};
struct Calibrated_a calibrated_acce ={0};

// float Ax;
// float Ay;
// float Az;

// float xMax= 1.01;
// float xMin= -0.98;

// float yMax= 0.99;
// float yMin= -1.01;

// float zMax= 1.01;
// float zMin= -1.00 ;

// float xOffset = (xMax + xMin)/2;
// float yOffset = (yMax + yMin)/2;
// float zOffset = (zMax + zMin)/2;

// float xScale = 2/(xMax - xMin);
// float yScale = 2/(yMax - yMin);
// float zScale = 2/(zMax - zMin);


float xOffset = 0.07;
float yOffset = -0.08;
float zOffset = -0.13;

float xScale = 1.00;
float yScale = 1.00;
float zScale = 0.98;


void calibrateAccelerometer() {
  Serial.println("Starting accelerometer calibration...");
  delay(2000);

  const int samples = 200;
  sensors_event_t a, g, t;

  // arrays to hold averages
  float x_avg[6], y_avg[6], z_avg[6];

  for (int face = 0; face < 6; face++) {
    Serial.print("Place IMU on face ");
    Serial.println(face);
    //Serial.println("Press any key when ready...");
    Serial.print("Place IMU on face, waiting 10 seconds...");
    delay(10000);
    float sumX = 0;
    float sumY = 0;
    float sumZ = 0;

    for (int i = 0; i < samples; i++) {
      icm.getEvent(&a, &g, &t);
      sumX += a.acceleration.x;
      sumY += a.acceleration.y;
      sumZ += a.acceleration.z;
      delay(10);
    }

    x_avg[face] = sumX / samples;
    y_avg[face] = sumY / samples;
    z_avg[face] = sumZ / samples;

    Serial.print("Avg: ");
    Serial.print(x_avg[face]); Serial.print(", ");
    Serial.print(y_avg[face]); Serial.print(", ");
    Serial.println(z_avg[face]);
  }

  // compute offsets and scales
  float x_offset = (x_avg[0] + x_avg[1]) / 2.0;
  float y_offset = (y_avg[2] + y_avg[3]) / 2.0;
  float z_offset = (z_avg[4] + z_avg[5]) / 2.0;

  float x_scale = (2.0 * 9.81) / fabs(x_avg[0] - x_avg[1]);
  float y_scale = (2.0 * 9.81) / fabs(y_avg[2] - y_avg[3]);
  float z_scale = (2.0 * 9.81) / fabs(z_avg[4] - z_avg[5]);

  Serial.println("Calibration results:");
  Serial.print("Offsets: ");
  Serial.print(x_offset); Serial.print(", ");
  Serial.print(y_offset); Serial.print(", ");
  Serial.println(z_offset);

  Serial.print("Scales: ");
  Serial.print(x_scale); Serial.print(", ");
  Serial.print(y_scale); Serial.print(", ");
  Serial.println(z_scale);

  // store them globally
  // xOffset = x_offset;
  // yOffset = y_offset;
  // zOffset = z_offset;

  // xScale = x_scale;
  // yScale = y_scale;
  // zScale = z_scale;

  Serial.println("Accelerometer calibration done!");
}

void calibrateGyro(uint16_t n_samples = 1000, uint16_t sample_delay_ms = 10) {
  Serial.println("Keep IMU still - Calibrating gyro...");
  delay(2000); 
  
  double sumX = 0, sumY = 0, sumZ = 0;
  sensors_event_t a, g, temp;

  for (uint16_t i = 0; i < n_samples; i++) {
    icm.getEvent(&a, &g, &temp);
    sumX += g.gyro.x;
    sumY += g.gyro.y;
    sumZ += g.gyro.z;
    delay(sample_delay_ms);
  }

  gyro_biases[0] = sumX / n_samples;
  gyro_biases[1] = sumY / n_samples;
  gyro_biases[2] = sumZ / n_samples;

  Serial.println("Gyro calibration done.");
  Serial.print("Biases: ");
  Serial.print(gyro_biases[0]); Serial.print(", ");
  Serial.print(gyro_biases[1]); Serial.print(", ");
  Serial.println(gyro_biases[2]);
}



void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit ICM20649 test!");
  
  // Try to initialize!
  if (!icm.begin_I2C()) {
    // if (!icm.begin_SPI(ICM_CS)) {
    // if (!icm.begin_SPI(ICM_CS, ICM_SCK, ICM_MISO, ICM_MOSI)) {

    Serial.println("Failed to find ICM20649 chip");
    while (1) {
      delay(10);
    }
  }

  Serial.println("ICM20649 Found!");
  icm.setAccelRange(ICM20649_ACCEL_RANGE_4_G);
  Serial.print("Accelerometer range set to: ");
  switch (icm.getAccelRange()) {
  case ICM20649_ACCEL_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case ICM20649_ACCEL_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case ICM20649_ACCEL_RANGE_16_G:
    Serial.println("+-16G");
    break;
  case ICM20649_ACCEL_RANGE_30_G:
    Serial.println("+-30G");
    break;
  }

  icm.setGyroRange(ICM20649_GYRO_RANGE_500_DPS);
  Serial.print("Gyro range set to: ");
  switch (icm.getGyroRange()) {
  case ICM20649_GYRO_RANGE_500_DPS:
    Serial.println("500 degrees/s");
    break;
  case ICM20649_GYRO_RANGE_1000_DPS:
    Serial.println("1000 degrees/s");
    break;
  case ICM20649_GYRO_RANGE_2000_DPS:
    Serial.println("2000 degrees/s");
    break;
  case ICM20649_GYRO_RANGE_4000_DPS:
    Serial.println("4000 degrees/s");
    break;
  }

  icm.setAccelRateDivisor(10);
  uint16_t accel_divisor = icm.getAccelRateDivisor();
  float accel_rate = 1125 / (1.0 + accel_divisor);

  

  Serial.print("Accelerometer data rate divisor set to: ");
  Serial.println(accel_divisor);
  Serial.print("Accelerometer data rate (Hz) is approximately: ");
  Serial.println(accel_rate);

  icm.setGyroRateDivisor(10);
  uint8_t gyro_divisor = icm.getGyroRateDivisor();
  float gyro_rate = 1100 / (1.0 + gyro_divisor);

  Serial.print("Gyro data rate divisor set to: ");
  Serial.println(gyro_divisor);
  Serial.print("Gyro data rate (Hz) is approximately: ");
  Serial.println(gyro_rate);
  Serial.println();
 //calibrateAccelerometer();
  calibrateGyro();

  filter.begin(100);

  microsPerReading = 1000000 / 100;
  microsPrevious = micros();

}

void loop() {

  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  unsigned long microsNow;
  float ax_r, ay_r, az_r;
  float gx, gy, gz;
  float ax_prev =0;
  float ay_prev =0;
  float az_prev= 0;
  float gx_dps, gy_dps, gz_dps;

    //  icm.getEvent(&accel, &gyro, &temp);
    // ax = accel.acceleration.x / 9.81; 
    // ay = accel.acceleration.y / 9.81;
    // az = accel.acceleration.z / 9.81;

    // Ax = xScale* (ax-xOffset);
    // Ay = yScale* (ay-yOffset);
    // Az = zScale* (az-zOffset);

    // Serial.print(Ax);
    // Serial.print(",");
    // Serial.print(Ay);
    // Serial.print(",");
    // Serial.print(Az);
    // Serial.println();

    // delay(50);

  float roll, pitch, heading;
  microsNow = micros();

  if (microsNow - microsPrevious >= microsPerReading){
    microsPrevious = microsNow;

    icm.getEvent(&accel, &gyro, &temp); ///get sensor data

  float ax_raw = xScale * (accel.acceleration.x - xOffset);
  float ay_raw = yScale * (accel.acceleration.y - yOffset);
  float az_raw = zScale * (accel.acceleration.z - zOffset);

  float alpha = 0.05; // smoothing factor (smaller = smoother)
  float ax = alpha * ax_prev + (1 - alpha) * ax_raw;
  float ay = alpha * ay_prev + (1 - alpha) * ay_raw;
  float az = alpha * az_prev + (1 - alpha) * az_raw;

// store for next iteration
  ax_prev = ax;
  ay_prev = ay;
  az_prev = az;

    // Ax = xScale* (ax - xOffset);
    // Ay = yScale* (ay - yOffset);
    // Az = zScale* (az - zOffset);

    gx = gyro.gyro.x - gyro_biases[0];
    gy = gyro.gyro.y - gyro_biases[1];
    gz = gyro.gyro.z - gyro_biases[2];

    // gx = gyro.gyro.x ;
    // gy = gyro.gyro.y ;
    // gz = gyro.gyro.z ;

    gx_dps = gx * (180. / PI);
    gy_dps = gy * (180. / PI);
    gz_dps = gz * (180. / PI);

    // ax_r = Ax * 9.81;
    // ay_r = Ay * 9.81;
    // az_r = Az * 9.81;
    filter.updateIMU(gx_dps, gy_dps, gz_dps, ax, ay, az);

    roll = filter.getRoll();
    pitch = filter.getPitch();
    heading = filter.getYaw();

    Serial.print(roll);
    Serial.print(",");
    Serial.print(pitch);
    Serial.print(",");
    Serial.print(heading);
    Serial.println();

    //microsPrevious = microsPrevious + microsPerReading;

  }


}


