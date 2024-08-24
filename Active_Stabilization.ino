// Libraries
#include <Wire.h>
#include <Servo.h>
#include <PID_v1.h>

// Motor Flag (PID calculation starts when flag is turned on to avoid feeding false data to the PID)
bool motorRunning = false;

// Servo Offsets (This has to be done manually everytime the gimbal is taken off of the servos)
const int servoxinit = 93, servoyinit = 83;

// PID Values
const double Kp = 0.1062, Ki = 0.27612, Kd = 0.010211538;

// Servo objects
Servo servoX;   // x-axis servo (outer gimbal)
Servo servoY;   // y-axis servo (inner gimbal)
Servo esc;      // JP Hobby 160A ESC

// PID variables for X axis
double setpointX = 0.0, inputX, outputX; // setpointX = what we want the pitch angle to be; inputX is the current pitch angle; and outputX feeds to servoX to correct offset)
PID pidX(&inputX, &outputX, &setpointX, Kp, Ki, Kd, DIRECT);

// PID variables for Y axis
double setpointY = 0.0, inputY, outputY; // setpointY = what we want the roll angle to be; inputY is the current roll angle; and outputY feeds to servoY to correct offset)
PID pidY(&inputY, &outputY, &setpointY, Kp, Ki, Kd, DIRECT);

// Gyroscope and accelerometer variables
float RateRoll, RatePitch, RateYaw;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
int RateCalibrationNumber;
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;
uint32_t LoopTimer;

// Kalman filter variables (predicted angles and uncertainities)
float KalmanAngleRoll=0, KalmanUncertaintyAngleRoll=2*2; // up to 2 degrees uncertainity
float KalmanAnglePitch=0, KalmanUncertaintyAnglePitch=2*2; // because rocket can never be completely flat

// Kalman Filter Output
float Kalman1DOutput[]={0,0};

void setup(void) {
  // Initialize the serial communication
  Serial.begin(115200);

  // Prevents code from locking up
  Wire.setWireTimeout(3000, true);

  // ESC setup
  pinMode(4, INPUT);        // ESC short wire
  // Serial.println(digitalRead(4)); 
  esc.attach(6, 900, 2400); // ESC long wire

  // MPU-6050 Connection and Calibration
  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  Wire.beginTransmission(0x68); 
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  Serial.println("\n");
  Serial.println("Begin Callibration, do not move\n");
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
  Serial.println("Callibration Finished\n");

  // Attach servos
  servoX.attach(9);  // x-axis servo (outer gimbal)
  servoY.attach(10); // y-axis servo (inner gimbal)
  
  // Set initial servo positions
  servoX.write(servoxinit);
  servoY.write(servoyinit);

  // Initialize PID controllers
  pidX.SetMode(AUTOMATIC);
  pidY.SetMode(AUTOMATIC);
  pidX.SetOutputLimits(-20,20);
  pidY.SetOutputLimits(-20,20);

  // Confirm setup completion
  Serial.println("Setup Ready, press A to turn on motor!");
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

  if (motorRunning) {
    // Calculates the rotation rates
    gyro_signals();
    RateRoll-=RateCalibrationRoll;
    RatePitch-=RateCalibrationPitch;
    RateYaw-=RateCalibrationYaw;

    // Starts the iteration for the Kalman filter with the roll and pitch angles 
    kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
    KalmanAngleRoll=Kalman1DOutput[0]; 
    KalmanUncertaintyAngleRoll=Kalman1DOutput[1];
    kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
    KalmanAnglePitch=Kalman1DOutput[0]; 
    KalmanUncertaintyAnglePitch=Kalman1DOutput[1];

    // Use filtered angles as input to PID
    inputX = KalmanAnglePitch; // inputX = KalmanAngleRoll;
    pidX.Compute();
    int servoPosX = constrain(servoxinit + outputX, servoxinit - 20, servoxinit + 20);
    servoX.write(servoPosX);

    inputY = KalmanAngleRoll; // inputY = KalmanAnglePitch;
    pidY.Compute();
    int servoPosY = constrain(servoyinit + outputY, servoyinit - 20, servoyinit + 20);
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
}

  // Calculates predicted angle and uncertainty using the Kalman equations; credit to Carbon Aeronautics
void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  KalmanState=KalmanState+0.004*KalmanInput;
  KalmanUncertainty=KalmanUncertainty + 0.004 * 0.004 * 4 * 4;
  float KalmanGain=KalmanUncertainty * 1/(1*KalmanUncertainty + 3 * 3);
  KalmanState=KalmanState+KalmanGain * (KalmanMeasurement-KalmanState);
  KalmanUncertainty=(1-KalmanGain) * KalmanUncertainty;

  // Kalman filter output
  Kalman1DOutput[0]=KalmanState; 
  Kalman1DOutput[1]=KalmanUncertainty;
}

// Read the rotation rates, acceleration, and angles from the MPU-6050; credit to Carbon Aeronautics
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
  AccX=(float)AccXLSB/4096-0.03; // accero calibration, this is done manually
  AccY=(float)AccYLSB/4096;
  AccZ=(float)AccZLSB/4096;
  AngleRoll=atan(AccY/sqrt(AccX*AccX+AccZ*AccZ))*1/(3.142/180);
  AnglePitch=-atan(AccX/sqrt(AccY*AccY+AccZ*AccZ))*1/(3.142/180);
}

// Starts motor
void startMotor() {
  Serial.println("Motor Begin");
  motorRunning = true;  // Set motor status flag
  set_esc_power(esc, 60);
  Serial.println("Motor Started");
}

// Stops motor
void stopMotor() {
  Serial.println("Stopping Motor");
  motorRunning = false;  // Reset motor status flag
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
