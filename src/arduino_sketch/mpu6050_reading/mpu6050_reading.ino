#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

void setup(void) {
  // Use a fast baud rate to prevent serial bottlenecking
  Serial.begin(115200);

  Serial.println("Initializing MPU6050...");

  Wire.begin(4, 0);

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip. Check wiring on D1/D2!");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  // Setup motion tracking ranges
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  
  // Filter out high-frequency vibrations (like motor noise)
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Print a CSV header
  Serial.println("time_ms,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z");
  delay(100);
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  unsigned long timestamp = millis();

  // Print data in CSV format: time, ax, ay, az, gx, gy, gz
  Serial.print(timestamp);
  Serial.print(",");
  Serial.print(a.acceleration.x);
  Serial.print(",");
  Serial.print(a.acceleration.y);
  Serial.print(",");
  Serial.print(a.acceleration.z);
  Serial.print(",");
  Serial.print(g.gyro.x);
  Serial.print(",");
  Serial.print(g.gyro.y);
  Serial.print(",");
  Serial.println(g.gyro.z);

  // 20ms delay = 50Hz publish rate
  delay(20); 
}