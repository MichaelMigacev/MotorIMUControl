#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <ESP32Servo.h>

Adafruit_MPU6050 mpu;
Servo myservo_x;
Servo myservo_y;
Servo myservo_z;

void setup(void)
{
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");
  delay(100);
  // Try to initialize!
  if (mpu.begin())
  {
    Serial.println("Failed to find MPU6050 chip");
    while (1)
    {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  myservo_x.attach(12, 500, 2500);
  myservo_y.attach(13, 500, 2500);
  myservo_z.attach(14, 500, 2500);

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange())
  {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange())
  {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth())
  {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  Serial.println("");
  delay(100);
}

void loop()
{
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Map acceleration.x from -10 to 10 to servo angle 0 to 180
  float accelX = a.acceleration.x;
  if (accelX < -10)
    accelX = -10;
  if (accelX > 10)
    accelX = 10;
  int servoAngle_x = round((accelX + 10) * 180.0 / 20.0);
  myservo_x.write(servoAngle_x);

  // Map acceleration.x from -10 to 10 to servo angle 0 to 180
  float accelY = a.acceleration.y;
  if (accelY < -10)
    accelY = -10;
  if (accelY > 10)
    accelY = 10;
  int servoAngle_y = round((accelY + 10) * 180.0 / 20.0);
  myservo_y.write(servoAngle_y);

  // Map acceleration.z from -10 to 10 to servo angle 0 to 180
  float accelZ = a.acceleration.z;
  if (accelZ < -10)
    accelZ = -10;
  if (accelZ > 10)
    accelZ = 10;
  int servoAngle_z = round((accelZ + 10) * 180.0 / 20.0);
  myservo_z.write(servoAngle_z);

  /* Print out the values */
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  Serial.print("Servo Angle X: ");
  Serial.println(servoAngle_x);
  Serial.print("Servo Angle Y: ");
  Serial.println(servoAngle_y);
  Serial.print("Servo Angle Z: ");
  Serial.println(servoAngle_z);

  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");

  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" degC");

  Serial.println("");
  delay(100);
}