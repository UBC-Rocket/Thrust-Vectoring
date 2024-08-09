// Libraries
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Servo.h>
#include <PID_v1.h>

// Servo objects
Servo servoX;
Servo servoY;
Servo esc_1;  // c

// servo offsets
int servoxinit = 94, servoyinit = 84;
// PID constants
double Kp = 1, Ki = 0.0, Kd = 0.0;

// PID variables for X axis
double setpointX = 0.0, inputX, outputX;
PID pidX(&inputX, &outputX, &setpointX, Kp, Ki, Kd, DIRECT);

// PID variables for Y axis
double setpointY = 0.0, inputY, outputY;
PID pidY(&inputY, &outputY, &setpointY, Kp, Ki, Kd, DIRECT);

Adafruit_MPU6050 mpu;

// Kalman filter variables
float KalmanAngleRoll=0, KalmanUncertaintyAngleRoll=2*2;
float KalmanAnglePitch=0, KalmanUncertaintyAnglePitch=2*2;
float Kalman1DOutput[]={0,0};

// Function to update Kalman filter
void kalman_1d(float &KalmanState, float &KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  KalmanState = KalmanState + 0.004 * KalmanInput;
  KalmanUncertainty = KalmanUncertainty + 0.004 * 0.004 * 4 * 4;
  float KalmanGain = KalmanUncertainty / (KalmanUncertainty + 3 * 3);
  KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
  KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;
  Kalman1DOutput[0] = KalmanState;
  Kalman1DOutput[1] = KalmanUncertainty;
}

void setup(void) {
  pinMode(4, INPUT);
  Serial.begin(9600);
  Serial.println(digitalRead(4));

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
  esc_1.attach(6, 900, 2400); // Attach ESC to pin 11 (or any PWM capable pin)

  // Set initial servo positions
  servoX.write(servoxinit);
  servoY.write(servoyinit);

  // Initialize PID controllers
  pidX.SetMode(AUTOMATIC);
  pidY.SetMode(AUTOMATIC);
  pidX.SetOutputLimits(-4,4);
  pidY.SetOutputLimits(-4,4);

  Serial.println("Setup Ready");

}

void loop() {
  // Check if stop command received
  if (Serial.available() > 0) {
    char command = Serial.read();
        if (command == 'A') {
          startMotor();
        } else if (command == 'F') {
          stopMotor();
        }
      }

  // Get new sensor events with the readings
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Apply Kalman filter to accelerometer and gyroscope data
  kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, g.gyro.x, a.acceleration.x);
  KalmanAngleRoll = Kalman1DOutput[0];
  KalmanUncertaintyAngleRoll = Kalman1DOutput[1];

  kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, g.gyro.y, a.acceleration.y);
  KalmanAnglePitch = Kalman1DOutput[0];
  KalmanUncertaintyAnglePitch = Kalman1DOutput[1];

  // Use filtered angles as input to PID
  // inputX = KalmanAngleRoll;
  inputX = a.acceleration.x;
  pidX.Compute();
  int servoPosX = constrain(servoxinit + 10*outputX, servoxinit - 10, servoxinit + 10);
  servoX.write(servoPosX);

  // inputY = KalmanAnglePitch;
  inputY = a.acceleration.y;
  pidY.Compute();
  int servoPosY = constrain(servoyinit - 10*outputY, servoyinit - 10, servoyinit + 10);
  servoY.write(servoPosY);

  // Outputs on terminal
  Serial.print("Kalman Roll Angle: ");
  Serial.print(KalmanAngleRoll);
  Serial.print(", Kalman Pitch Angle: ");
  Serial.println(KalmanAnglePitch);

  // Serial.print("PID Input X: ");
  // Serial.print(inputX);
  // Serial.print(", PID Output X: ");
  // Serial.print(outputX);
  // Serial.print(", Servo Position X: ");
  // Serial.println(servoPosX);

  // Serial.print("PID Input Y: ");
  // Serial.print(inputY);
  // Serial.print(", PID Output Y: ");
  // Serial.print(outputY);
  // Serial.print(", Servo Position Y: ");
  // Serial.println(servoPosY);
  
  delay(10);
}

void startMotor() {
  Serial.println("Motor Begin");
  set_esc_power(esc_1, 60);
  Serial.println("Motor Started");
}

void stopMotor() {
  Serial.println("Stopping Motor");
  set_esc_power(esc_1, 0);
  Serial.println("Motor Stopped");
}

int set_esc_power (Servo esc, int power) {
  power = constrain(power, 0, 100);
  int signal_min = 900;
  int signal_max = 2400;
  int signal_output = map(power, 0, 100, signal_min, signal_max); //map(value, fromLow, fromHigh, toLow, toHigh)
  esc.writeMicroseconds(signal_output);
  return signal_output;
}