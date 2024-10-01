#include <Wire.h>

// Sensor I2C addresses
#define LSM6DS33_ADDRESS  0x6B  // Gyro and accelerometer
#define LIS3MDL_ADDRESS   0x1E  // Magnetometer
#define LPS25H_ADDRESS    0x5D  // Barometer

// Conversion factors
const float ACCEL_SCALE = 0.061;  // mg/LSB for ±2g at 16-bit resolution
const float GYRO_SCALE = 8.75;    // mdps/LSB for ±245 dps at 16-bit resolution
const float MAG_SCALE = 0.14;     // µT/LSB for ±4 gauss at 16-bit resolution
const float PRESSURE_SCALE = 4096.0;  // Pressure conversion from raw

void setup() {
  Wire.begin();
  Serial.begin(9600);

  // Initialize LSM6DS33 (Gyro + Accelerometer)
  Wire.beginTransmission(LSM6DS33_ADDRESS);
  Wire.write(0x10); // CTRL1_XL register
  Wire.write(0x80); // Set to 1.66 kHz, 2g, 100 Hz filter
  Wire.endTransmission();
  
  Wire.beginTransmission(LSM6DS33_ADDRESS);
  Wire.write(0x11); // CTRL2_G register
  Wire.write(0x80); // Set to 1.66 kHz, 245 dps
  Wire.endTransmission();

  // Initialize LIS3MDL (Magnetometer)
  Wire.beginTransmission(LIS3MDL_ADDRESS);
  Wire.write(0x20); // CTRL_REG1
  Wire.write(0x70); // Set to 10 Hz, ultra-high-performance mode
  Wire.endTransmission();
  
  // Initialize LPS25H (Barometer)
  Wire.beginTransmission(LPS25H_ADDRESS);
  Wire.write(0x20); // CTRL_REG1
  Wire.write(0x90); // Power on, output data rate 1 Hz
  Wire.endTransmission();
  
  // Check LSM6DS33 WHO_AM_I to confirm sensor is ready
  Wire.beginTransmission(LSM6DS33_ADDRESS);
  Wire.write(0x0F); // WHO_AM_I register
  Wire.endTransmission();
  Wire.requestFrom(LSM6DS33_ADDRESS, 1);
  if (Wire.available()) {
    byte whoAmI = Wire.read();
    Serial.print("LSM6DS33 WHO_AM_I: ");
    Serial.println(whoAmI, HEX);
  }
}

void loop() {
  readLSM6DS33();  // Reads accelerometer and gyroscope data
  readLIS3MDL();   // Reads magnetometer data
  readLPS25H();    // Reads barometer data
  delay(1000);
}

void readLSM6DS33() {
  // Reading accelerometer data
  Wire.beginTransmission(LSM6DS33_ADDRESS);
  Wire.write(0x28); // OUTX_L_XL register (start reading accelerometer)
  Wire.endTransmission(false);
  Wire.requestFrom(LSM6DS33_ADDRESS, 6); // Request 6 bytes of data (X, Y, Z)

  if (Wire.available() == 6) {
    int16_t ax = Wire.read() | (Wire.read() << 8);
    int16_t ay = Wire.read() | (Wire.read() << 8);
    int16_t az = Wire.read() | (Wire.read() << 8);

    // Convert raw values to g's using scaling factor
    float accelX = ax * ACCEL_SCALE / 1000.0;  // mg to g
    float accelY = ay * ACCEL_SCALE / 1000.0;
    float accelZ = az * ACCEL_SCALE / 1000.0;

    Serial.print("Accel (g) X: "); Serial.print(accelX);
    Serial.print(", Y: "); Serial.print(accelY);
    Serial.print(", Z: "); Serial.println(accelZ);
  } else {
    Serial.println("Failed to read accelerometer data");
  }

  // Reading gyroscope data (angular velocity)
  Wire.beginTransmission(LSM6DS33_ADDRESS);
  Wire.write(0x22); // OUTX_L_G register (start reading gyroscope)
  Wire.endTransmission(false);
  Wire.requestFrom(LSM6DS33_ADDRESS, 6); // Request 6 bytes (X, Y, Z)

  if (Wire.available() == 6) {
    int16_t gx = Wire.read() | (Wire.read() << 8);
    int16_t gy = Wire.read() | (Wire.read() << 8);
    int16_t gz = Wire.read() | (Wire.read() << 8);

    // Convert raw values to degrees per second (dps) using scaling factor
    float gyroX = gx * GYRO_SCALE / 1000.0;  // mdps to dps
    float gyroY = gy * GYRO_SCALE / 1000.0;
    float gyroZ = gz * GYRO_SCALE / 1000.0;

    Serial.print("Gyro (dps) X: "); Serial.print(gyroX);
    Serial.print(", Y: "); Serial.print(gyroY);
    Serial.print(", Z: "); Serial.println(gyroZ);
  } else {
    Serial.println("Failed to read gyroscope data");
  }
}

void readLIS3MDL() {
  Wire.beginTransmission(LIS3MDL_ADDRESS);
  Wire.write(0x28); // OUT_X_L register
  Wire.endTransmission(false);
  Wire.requestFrom(LIS3MDL_ADDRESS, 6); // Request 6 bytes (X, Y, Z)

  if (Wire.available() == 6) {
    int16_t mx = Wire.read() | (Wire.read() << 8);
    int16_t my = Wire.read() | (Wire.read() << 8);
    int16_t mz = Wire.read() | (Wire.read() << 8);

    // Convert raw values to microteslas (µT) using scaling factor
    float magnetX = mx * MAG_SCALE;
    float magnetY = my * MAG_SCALE;
    float magnetZ = mz * MAG_SCALE;

    Serial.print("Magnet (µT) X: "); Serial.print(magnetX);
    Serial.print(", Y: "); Serial.print(magnetY);
    Serial.print(", Z: "); Serial.println(magnetZ);
  } else {
    Serial.println("Failed to read magnetometer data");
  }
}

void readLPS25H() {
  Wire.beginTransmission(LPS25H_ADDRESS);
  Wire.write(0x28); // PRESS_OUT_XL register
  Wire.endTransmission(false);
  Wire.requestFrom(LPS25H_ADDRESS, 3); // Request 3 bytes (pressure data)

  if (Wire.available() == 3) {
    uint32_t pressure = Wire.read() | (Wire.read() << 8) | (Wire.read() << 16);

    // Convert raw pressure data to hPa (divide by scaling factor)
    float pressure_hPa = pressure / PRESSURE_SCALE;
    Serial.print("Pressure (hPa): "); Serial.println(pressure_hPa);
  } else {
    Serial.println("Failed to read pressure data");
  }
}
