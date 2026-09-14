#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

void setup(void) {
  // Use a fast baud rate to prevent serial bottlenecking
  Serial.begin(115200);
  
  // Wait for serial monitor to open (optional, good for debugging)
  while (!Serial)
    delay(10);

  Serial.println("Initializing MPU6050...");

  // Wire.begin() automatically uses pins 20 (SDA) and 21 (SCL) on the Mega
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip. Check wiring!");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  // Setup motion tracking ranges
  // Lower range = higher precision, but might clip on sudden impacts
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  
  // Filter out high-frequency vibrations (like motor noise)
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Print a CSV header
  Serial.println("time_ms,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z");
  delay(100);
}

void loop() {
  // Get new sensor events with the most recent readings
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Capture the timestamp for this specific reading
  unsigned long timestamp = millis();

  // Print data in CSV format: time, ax, ay, az, gx, gy, gz
  // Accelerometer outputs in m/s^2. Gyro outputs in rad/s.
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

  // Delay to control the publish rate (e.g., 20ms = 50Hz)
  delay(20); 
}