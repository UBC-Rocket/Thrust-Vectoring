// Libraries
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Servo.h>
#include <PID_v1.h>

// Servo Offsets
const int servoxinit = 94, servoyinit = 84;

// PID Values
const double Kp = 0.0588, Ki = 0.16, Kd = 0.0;

// Servo objects
Servo servoX;   // x-axis servo (outer gimbal)
Servo servoY;   // y-axis servo (inner gimbal)
Servo esc;      // JP Hobby 160A ESC

// PID variables for X axis
double setpointX = 0.0, inputX, outputX;
PID pidX(&inputX, &outputX, &setpointX, Kp, Ki, Kd, DIRECT);

// PID variables for Y axis
double setpointY = 0.0, inputY, outputY;
PID pidY(&inputY, &outputY, &setpointY, Kp, Ki, Kd, DIRECT);

// MPU-6050 Variable
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
  // Initialize the serial communication
  Serial.begin(115200);

  // Prevents code from locking up
  Wire.setWireTimeout(3000, true);

  // ESC setup
  pinMode(4, INPUT);        // ESC short wire
  // Serial.println(digitalRead(4)); 
  esc.attach(6, 900, 2400); // ESC long wire

  // MPU-6050 Connection Test
  Serial.println("Adafruit MPU-6050 test!");
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU-6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU-6050 Found!");

  // Initialize MPU-6050
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Attach servos
  servoX.attach(9);  // x-axis servo (outer gimbal)
  servoY.attach(10); // y-axis servo (inner gimbal)
  
  // Set initial servo positions
  servoX.write(servoxinit);
  servoY.write(servoyinit);

  // Initialize PID controllers
  pidX.SetMode(AUTOMATIC);
  pidY.SetMode(AUTOMATIC);
  pidX.SetOutputLimits(-4,4);
  pidY.SetOutputLimits(-4,4);

  // Confirm setup completion
  Serial.println("Setup Ready");
}

void loop() {

  // Motor On / Off control via serial
  if (Serial.available() > 0) {
    char command = Serial.read();
        if (command == 'A') {
          startMotor(); // starts motor
        } else if (command == 'F') {
          stopMotor();  // stops motor
        }
      }

  // Retrieve latest MPU-6050 data
  sensors_event_t a, g, temp; // a = accelerometer data, g = gyroscope data, temp = temperature data
  mpu.getEvent(&a, &g, &temp);

  // Apply Kalman filter to accelerometer and gyroscope data
  kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, g.gyro.x, a.acceleration.x);
  KalmanAngleRoll = Kalman1DOutput[0];
  KalmanUncertaintyAngleRoll = Kalman1DOutput[1];

  kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, g.gyro.y, a.acceleration.y);
  KalmanAnglePitch = Kalman1DOutput[0];
  KalmanUncertaintyAnglePitch = Kalman1DOutput[1];

  // Use filtered angles as input to PID
  inputX = a.acceleration.x; // inputX = KalmanAngleRoll;
  pidX.Compute();
  int servoPosX = constrain(servoxinit - 10*outputX, servoxinit - 12, servoxinit + 12);
  servoX.write(servoPosX);

  inputY = a.acceleration.y; // inputY = KalmanAnglePitch;
  pidY.Compute();
  int servoPosY = constrain(servoyinit + 10*outputY, servoyinit - 12, servoyinit + 12);
  servoY.write(servoPosY);

  // Outputs on terminal
  // Serial.print("Kalman Roll Angle: ");
  // Serial.print(KalmanAngleRoll);
  // Serial.print(", Kalman Pitch Angle: ");
  // Serial.println(KalmanAnglePitch);

  // Serial.print("PID Input X: ");
  // Serial.print(inputX);
  // Serial.print(", PID Output X: ");
  // Serial.print(outputX);
  Serial.print(", Servo Position X: ");
  Serial.println(servoPosX);

  // Serial.print("PID Input Y: ");
  // Serial.print(inputY);
  // Serial.print(", PID Output Y: ");
  // Serial.print(outputY);
  Serial.print(", Servo Position Y: ");
  Serial.println(servoPosY);

  delay(20);
}

// Starts motor
void startMotor() {
  Serial.println("Motor Begin");
  set_esc_power(esc, 60);
  Serial.println("Motor Started");
}

// Stops motor
void stopMotor() {
  Serial.println("Stopping Motor");
  set_esc_power(esc, 0);
  Serial.println("Motor Stopped");
}

// Sets the power of the ESC fom 0 to 100
int set_esc_power (Servo esc, int power) {
  power = constrain(power, 0, 100);
  int signal_min = 900; // PWM Period
  int signal_max = 2400; // PWM Period
  int signal_output = map(power, 0, 100, signal_min, signal_max); //map(value, fromLow, fromHigh, toLow, toHigh)
  esc.writeMicroseconds(signal_output);
  return signal_output;
}