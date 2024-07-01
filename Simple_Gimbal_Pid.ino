#include <Servo.h>

// create servo objects
Servo xaxis;
Servo yaxis;

// PID variables
double Setpoint, Input, Output;
double kp = 1.0, ki = 0.5, kd = 0.1;  

// PID setup
PID TVid(&Input, &Output, &Setpoint, kp, ki, kd, DIRECT);

// Calibration variables
const int feedbackPin = A0; 
const int centerFeedbackValue = 288; 

void setup() {
  xaxis.attach(9);
  yaxis.attach(10);

  // initialize PID variables
  Input = analogRead(feedbackPin); 
  Setpoint = centerFeedbackValue; 

  // turn the PID on
  TVid.SetMode(AUTOMATIC);

  Serial.begin(9600);
}

void loop() {
  // read feedback from the servo position
  Input = analogRead(feedbackPin);

  // compute the PID output
  TVid.Compute();

  // write the PID output to the servo
  int servoAngle = map(Output, 0, 1023, 0, 180);
  xaxis.write(servoAngle);

  // print debug information
  Serial.print("Input: ");
  Serial.print(Input);
  Serial.print(" | Output: ");
  Serial.print(Output);
  Serial.print(" | Setpoint: ");
  Serial.print(Setpoint);
  Serial.print(" | Servo Angle: ");
  Serial.println(servoAngle);

  delay(100); 
}
