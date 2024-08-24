#include <Servo.h>

Servo servox;  // servo object
Servo servoy;

void setup() {
  servox.attach(9);  // x axis servo, outer gimbal
  servoy.attach(10); // y axis servo, inner gimbal
}

void loop() {
    servox.write(90); // tell servo to go to position
    servoy.write(90);              
  }
}
