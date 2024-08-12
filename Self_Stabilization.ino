// Libraries
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

// Kalman filter variables
float RateRoll, RatePitch, RateYaw;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
int RateCalibrationNumber;
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;
uint32_t LoopTimer;
float KalmanAngleRoll=0, KalmanUncertaintyAngleRoll=2*2;
float KalmanAnglePitch=0, KalmanUncertaintyAnglePitch=2*2;
float Kalman1DOutput[]={0,0};

// Function to update Kalman filter, credit to Carbon Aeronautics
void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  KalmanState=KalmanState+0.004*KalmanInput;
  KalmanUncertainty=KalmanUncertainty + 0.004 * 0.004 * 4 * 4;
  float KalmanGain=KalmanUncertainty * 1/(1*KalmanUncertainty + 3 * 3);
  KalmanState=KalmanState+KalmanGain * (KalmanMeasurement-KalmanState);
  KalmanUncertainty=(1-KalmanGain) * KalmanUncertainty;
  Kalman1DOutput[0]=KalmanState; 
  Kalman1DOutput[1]=KalmanUncertainty;
}

// FUnction to callibrate gyro and accero
void gyro_signals(void) {
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(); 
  Wire.requestFrom(0x68,6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();
  Wire.beginTransmission(0x68);
  Wire.write(0x1B); 
  Wire.write(0x8);
  Wire.endTransmission();     
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);
  int16_t GyroX=Wire.read()<<8 | Wire.read();
  int16_t GyroY=Wire.read()<<8 | Wire.read();
  int16_t GyroZ=Wire.read()<<8 | Wire.read();
  RateRoll=(float)GyroX/65.5;
  RatePitch=(float)GyroY/65.5;
  RateYaw=(float)GyroZ/65.5;
  AccX=(float)AccXLSB/4096-0.03;
  AccY=(float)AccYLSB/4096;
  AccZ=(float)AccZLSB/4096;
  AngleRoll=atan(AccY/sqrt(AccX*AccX+AccZ*AccZ))*1/(3.142/180);
  AnglePitch=-atan(AccX/sqrt(AccY*AccY+AccZ*AccZ))*1/(3.142/180);
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
  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  Wire.beginTransmission(0x68); 
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  for (RateCalibrationNumber=0; RateCalibrationNumber<2000; RateCalibrationNumber ++) {
    gyro_signals();
    RateCalibrationRoll+=RateRoll;
    RateCalibrationPitch+=RatePitch;
    RateCalibrationYaw+=RateYaw;
    delay(1);
  }
  RateCalibrationRoll/=2000;
  RateCalibrationPitch/=2000;
  RateCalibrationYaw/=2000;
  LoopTimer=micros();

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

  gyro_signals();
  RateRoll-=RateCalibrationRoll;
  RatePitch-=RateCalibrationPitch;
  RateYaw-=RateCalibrationYaw;
  kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
  KalmanAngleRoll=Kalman1DOutput[0]; 
  KalmanUncertaintyAngleRoll=Kalman1DOutput[1];
  kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
  KalmanAnglePitch=Kalman1DOutput[0]; 
  KalmanUncertaintyAnglePitch=Kalman1DOutput[1];

  // Use filtered angles as input to PID
  inputX = KalmanAnglePitch; // inputX = KalmanAngleRoll;
  pidX.Compute();
  int servoPosX = constrain(servoxinit - 10*outputX, servoxinit - 12, servoxinit + 12);
  servoX.write(servoPosX);

  inputY = KalmanAngleRoll; // inputY = KalmanAnglePitch;
  pidY.Compute();
  int servoPosY = constrain(servoyinit + 10*outputY, servoyinit - 12, servoyinit + 12);
  servoY.write(servoPosY);

  Serial.print("Roll Angle [°] ");
  Serial.print(KalmanAngleRoll);
  Serial.print(" Pitch Angle [°] ");
  Serial.println(KalmanAnglePitch);

  // Serial.print("PID Input X: ");
  // Serial.print(inputX);
  // Serial.print(", PID Output X: ");
  // Serial.print(outputX);
  // Serial.print(", Servo Position X: ");
  // Serial.println(servoPosX);

  // // Serial.print("PID Input Y: ");
  // // Serial.print(inputY);
  // // Serial.print(", PID Output Y: ");
  // // Serial.print(outputY);
  // Serial.print(", Servo Position Y: ");
  // Serial.println(servoPosY);

  while (micros() - LoopTimer < 4000);
  LoopTimer=micros();
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
