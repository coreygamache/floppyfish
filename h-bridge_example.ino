//define pins
//each motor has 2 output pins; one for forward and one for reverse
//make sure only one is being used at a time for each motor and the other is set to LOW or you will burn out the motor
const int Motor1Out1 = 8;
const int Motor1Out2 = 9;
const int Motor2Out1 = 10;
const int Motor2Out2 = 11;

//other constants
const int delayTime = 1000;

void setup()
{
  //set pin modes for motor pins
  pinMode(Motor1Out1, OUTPUT);
  pinMode(Motor1Out2, OUTPUT);
  pinMode(Motor2Out1, OUTPUT);
  pinMode(Motor2Out2, OUTPUT);
  
  //initialize serial communication
  Serial.flush();
  Serial.begin(9600);
  
  //start motors
  digitalWrite(Motor1Out1, HIGH); //using digitalWrite to control a motor only lets you either turn it on (at max power) or off. HIGH is max power
  digitalWrite(Motor1Out2, LOW); //LOW is off
  analogWrite(Motor2Out1, 90); //using analogWrite gives you control of motor speed. the number is a value from 0-255 that varies the output voltage to the motor.
  digitalWrite(Motor2Out2, LOW);
}

void loop()
{
  delay(delayTime);
}