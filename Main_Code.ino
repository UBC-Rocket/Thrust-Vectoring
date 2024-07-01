// Libraries
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Servo.h>
#include <PID_v1.h>

// Servo objects
Servo servoX;
Servo servoY;

// PID constants
double Kp = 1.0, Ki = 0.0, Kd = 0.1;

// PID variables for X axis
double setpointX = 0.0, inputX, outputX;
PID pidX(&inputX, &outputX, &setpointX, Kp, Ki, Kd, DIRECT);

// PID variables for Y axis
double setpointY = 0.0, inputY, outputY;
PID pidY(&inputY, &outputY, &setpointY, Kp, Ki, Kd, DIRECT);

Adafruit_MPU6050 mpu;

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println("Adafruit MPU6050 test!");

  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Attach servos
  servoX.attach(9);  // Attach to pin 9
  servoY.attach(10); // Attach to pin 10

  // Set initial servo positions
  servoX.write(90);
  servoY.write(90);

  // Initialize PID controllers
  pidX.SetMode(AUTOMATIC);
  pidY.SetMode(AUTOMATIC);

  Serial.println("");
  delay(100);
}

void loop() {
  // Get new sensor events with the readings
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Acceleration Values
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.println(" m/s^2");

  // PID calculations for X axis
  inputX = a.acceleration.x; // Using acceleration as input
  pidX.Compute();
  int servoPosX = constrain(90 + outputX, 0, 180);
  servoX.write(servoPosX);

  // PID calculations for Y axis
  inputY = a.acceleration.y; // Using acceleration as input
  pidY.Compute();
  int servoPosY = constrain(90 + outputY, 0, 180);
  servoY.write(servoPosY);

  // outputs on terminal
  Serial.print("PID Input X: ");
  Serial.print(inputX);
  Serial.print(", PID Output X: ");
  Serial.print(outputX);
  Serial.print(", Servo Position X: ");
  Serial.println(servoPosX);

  Serial.print("PID Input Y: ");
  Serial.print(inputY);
  Serial.print(", PID Output Y: ");
  Serial.print(outputY);
  Serial.print(", Servo Position Y: ");
  Serial.println(servoPosY);

  Serial.println("");
  delay(500);
}
