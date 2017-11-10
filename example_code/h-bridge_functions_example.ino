const int fwd1 = 3;
const int rev1 = 5;
const int fwd2 = 6;
const int rev2 = 9;

void setup() {
  pinMode(fwd1, OUTPUT);
  pinMode(rev1, OUTPUT);
  pinMode(fwd2, OUTPUT);
  pinMode(rev2, OUTPUT);

}

void loop() {
  //motorFWD(1, 5000);
  delay(1000);
}

//motor = motor # to turn on, time = time to run forward
void motorFWD(int motor, int time) {
  if (motor == 1)
    digitalWrite(fwd1, HIGH);
  else if (motor == 2)
    digitalWrite(fwd2, HIGH);
  delay(time);
  motorStop(1);
}

void motorREV() {
}

void motorStop(int motor) {
  if (motor == 1) {
    digitalWrite(fwd1, LOW);
    digitalWrite(rev1, LOW);
  }
  else if (motor == 2) {
    digitalWrite(fwd2, LOW);
    digitalWrite(rev2, LOW);  
  }
}

