#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_BMP280.h>
#include <MadgwickAHRS.h>
#include <SPI.h>
#include <SD.h>

Adafruit_LSM6DSOX sox;
Adafruit_BMP280 bmp;
Madgwick filter;

File flightFile;

unsigned long lastTime = 0;
const unsigned long samplePeriod = 9615; // 104 Hz

unsigned long lastFlush = 0;
const unsigned long flushPeriod = 1000; // flush every second


float seaLevelPressure = 1013.25;

const int SD_CS = 5;

// Calibration values
float gyroBiasX = 0;
float gyroBiasY = 0;
float gyroBiasZ = 0;

float groundAltitude = 0;

String createFilename() {

  for (int i = 0; i < 100; i++) {

    String filename = "/FLIGHT";

    if (i < 10) {
      filename += "0";
    }

    filename += String(i);
    filename += ".CSV";

    if (!SD.exists(filename)) {
      return filename;
    }
  }

  return "/FLIGHT99.CSV";
}

void setup() {

  Serial.begin(115200);
  delay(500);
  Serial.println("Starting Flight Computer...");

  // Initialize SD card
  if (!SD.begin(SD_CS)) {
    Serial.println("SD failed!");
    while(1);
  }

  Serial.println("SD initialized");

  String filename = createFilename();

  Serial.print("Logging to: ");
  Serial.println(filename);

  flightFile = SD.open(filename, FILE_WRITE);

  if (!flightFile) {
    Serial.println("File open failed!");
    while(1);
  }

  // CSV header
  flightFile.println(
    "timestamp,qw,qx,qy,qz,altitude,ax,ay,az,gx,gy,gz,temp"
  );

  flightFile.flush();

  // Initialize IMU
  if (!sox.begin_I2C()) {
    Serial.println("Couldn't find LSM6DSOX!");
    while(1);
  }

  Serial.println("LSM6DSOX Found!");

  // Initialize BMP280
  if (!bmp.begin(0x77)) { 
    Serial.println("Couldn't find BMP280!");
    while(1);
  }

  Serial.println("BMP280 Found!");

  filter.begin(104);

  sox.setAccelRange(LSM6DS_ACCEL_RANGE_16_G);
  sox.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);

  sox.setAccelDataRate(LSM6DS_RATE_104_HZ);
  sox.setGyroDataRate(LSM6DS_RATE_104_HZ);

  unsigned long calibStart = millis();
  unsigned long calibLast = micros();

  int samples = 0;

  while (millis() - calibStart < 3000) {
    if (micros() - calibLast >= samplePeriod) {
      calibLast += samplePeriod;

      sensors_event_t accel;
      sensors_event_t gyro;
      sensors_event_t temp;

      sox.getEvent(&accel, &gyro, &temp);

      // Convert gyro to deg/s
      float gx = gyro.gyro.x * 180.0 / PI;
      float gy = gyro.gyro.y * 180.0 / PI;
      float gz = gyro.gyro.z * 180.0 / PI;

      // Accumulate gyro bias
      gyroBiasX += gx;
      gyroBiasY += gy;
      gyroBiasZ += gz;

      // Accumulate ground altitude
      groundAltitude += bmp.readAltitude(seaLevelPressure);

      // Let Madgwick settle
      filter.updateIMU(
        gx,
        gy,
        gz,
        accel.acceleration.x,
        accel.acceleration.y,
        accel.acceleration.z
      );

      samples++;
    }
  }

  // Compute averages
  gyroBiasX /= samples;
  gyroBiasY /= samples;
  gyroBiasZ /= samples;

  groundAltitude /= samples;

  Serial.println("Calibration complete.");

  Serial.print("Gyro Bias X: ");
  Serial.println(gyroBiasX);

  Serial.print("Gyro Bias Y: ");
  Serial.println(gyroBiasY);

  Serial.print("Gyro Bias Z: ");
  Serial.println(gyroBiasZ);

  Serial.print("Ground Altitude: ");
  Serial.println(groundAltitude);

  // Reset timers
  lastTime = micros();
  lastFlush = millis();

  Serial.println("Flight Computer Ready!");
}

////////////////////////////////////////////////


void loop() {


  if (micros() - lastTime >= samplePeriod) {

    lastTime += samplePeriod;
    unsigned long timestamp = lastTime;

    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;

    sox.getEvent(&accel, &gyro, &temp);

    // Convert gyro radians/s to deg/s
    float gx = gyro.gyro.x * 180 / PI - gyroBiasX;
    float gy = gyro.gyro.y * 180 / PI - gyroBiasY;
    float gz = gyro.gyro.z * 180 / PI - gyroBiasZ;

    filter.updateIMU(
      gx,
      gy,
      gz,
      accel.acceleration.x,
      accel.acceleration.y,
      accel.acceleration.z
    );

    float altitude = bmp.readAltitude(seaLevelPressure) - groundAltitude;

    // Write CSV line
    flightFile.print(timestamp);
    flightFile.print(",");

    flightFile.print(filter.getQ0());
    flightFile.print(",");

    flightFile.print(filter.getQ1());
    flightFile.print(",");

    flightFile.print(filter.getQ2());
    flightFile.print(",");

    flightFile.print(filter.getQ3());
    flightFile.print(",");

    flightFile.print(altitude);
    flightFile.print(",");

    flightFile.print(accel.acceleration.x);
    flightFile.print(",");

    flightFile.print(accel.acceleration.y);
    flightFile.print(",");

    flightFile.print(accel.acceleration.z);
    flightFile.print(",");

    flightFile.print(gx);
    flightFile.print(",");

    flightFile.print(gy);
    flightFile.print(",");

    flightFile.print(gz);
    flightFile.print(",");

    flightFile.println(temp.temperature);

    // Save to SD periodically
    if (millis() - lastFlush >= flushPeriod) {

      flightFile.flush();
      lastFlush = millis();

      Serial.println("SD flushed");

    }
  }
}