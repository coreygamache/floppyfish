//include the motor class files we created
#include <Motor.h>

//create motor variables of type "Motor" (the class we created), providing the forward pin and reverse pin as arguments
Motor motor1(3, 5);
Motor motor2(6, 9);

void setup() {
  
}

void loop() {
  motor1.forward();
  motor2.reverse();
  delay(1000);
  motor1.stop();
  motor2.stop();
  delay(1000);
  motor1.forward(500);
  motor2.reverse(1000);
  delay(5000);
}