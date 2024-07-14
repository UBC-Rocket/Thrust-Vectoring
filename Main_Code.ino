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
double Kp = 5.0, Ki = 1.0, Kd = 1.0;

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

void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  KalmanState=KalmanState+0.004*KalmanInput;
  KalmanUncertainty=KalmanUncertainty + 0.004 * 0.004 * 4 * 4;
  float KalmanGain=KalmanUncertainty * 1/(1*KalmanUncertainty + 3 * 3);
  KalmanState=KalmanState+KalmanGain * (KalmanMeasurement-KalmanState);
  KalmanUncertainty=(1-KalmanGain) * KalmanUncertainty;
  Kalman1DOutput[0]=KalmanState; 
  Kalman1DOutput[1]=KalmanUncertainty;
}

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
  servoY.write(84);

  // Initialize PID controllers
  pidX.SetMode(AUTOMATIC);
  pidY.SetMode(AUTOMATIC);
  pidX.SetOutputLimits(-4,4);
  pidY.SetOutputLimits(-4,4);

  Serial.println("");
  delay(100);
}

void loop() {
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
  inputX = KalmanAngleRoll;
  pidX.Compute();
  int servoPosX = constrain(90 + 5*outputX, 0, 180);
  servoX.write(servoPosX);

  inputY = KalmanAnglePitch;
  pidY.Compute();
  int servoPosY = constrain(84 + 5*outputY, 0, 180);
  servoY.write(servoPosY);

  // outputs on terminal
  Serial.print("Kalman Roll Angle: ");
  Serial.print(KalmanAngleRoll);
  Serial.print(", Kalman Pitch Angle: ");
  Serial.println(KalmanAnglePitch);

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
  delay(100);
}
