//define pins
const int Motor1In1 = 3;
const int Motor1In2 = 4;
const int Motor2In1 = 2;
const int Motor2In2 = 5;
const int Motor1Out1 = 8;
const int Motor1Out2 = 9;
const int Motor2Out1 = 10;
const int Motor2Out2 = 11;

//other constants
const int delayTime = 1000;
const int eventsPerRev1 = 228;
const int eventsPerRev2 = 228;

//define counters
volatile unsigned long counter = 0;
volatile unsigned long count1 = 0;
volatile unsigned long count2 = 0;
volatile float rotations1 = 0;
volatile float rotations2 = 0;

void setup()
{
  //set pin modes for motor pins
  pinMode(Motor1In1, INPUT);
  pinMode(Motor1In2, INPUT);
  pinMode(Motor2In1, INPUT);
  pinMode(Motor2In2, INPUT);
  pinMode(Motor1Out1, OUTPUT);
  pinMode(Motor1Out2, OUTPUT);
  pinMode(Motor2Out1, OUTPUT);
  pinMode(Motor2Out2, OUTPUT);
  digitalWrite(Motor1In1, HIGH);
  digitalWrite(Motor1In2, HIGH);
  digitalWrite(Motor2In1, HIGH);
  digitalWrite(Motor2In2, HIGH);
  
  //initialize hardware interrupts
  attachInterrupt(digitalPinToInterrupt(Motor1In1), Motor1Event, CHANGE); //pin 3
  attachInterrupt(digitalPinToInterrupt(Motor2In1), Motor2Event, CHANGE); //pin 2
  
  //initialize serial communication
  Serial.flush();
  Serial.begin(9600);
  
  //start motors
  digitalWrite(Motor1Out1, HIGH);
  digitalWrite(Motor1Out2, LOW);
  digitalWrite(Motor2Out1, HIGH);
  digitalWrite(Motor2Out2, LOW);
}

void loop()
{
  Serial.print("Time: ");
  Serial.print(counter * delayTime / 1000);
  Serial.println(" s");
  
  rotations1 += float(count1) / (eventsPerRev1 * 2);
  count1 = 0;
  Serial.print("Motor 1 Rotations: ");
  Serial.println(rotations1);
  
  rotations2 += float(count2) / (eventsPerRev2 * 2);
  count2 = 0;
  Serial.print("Motor 2 Rotations: ");
  Serial.println(rotations2);
  Serial.println();
  counter++;
  delay(delayTime);
}

//encoder event functions
void Motor1Event() {
  count1++;
  /*Serial.println("motor1 event");
  if (digitalRead(Motor1In1) == HIGH) {
    if (digitalRead(Motor1In2) == LOW) {
      count1++;
    }
    else {
      count1--;
    }
  }
  else {
    if (digitalRead(Motor1In2) == LOW) {
      count1--;
    }
    else {
      count1++;
    }
  }*/
}
                 
void Motor2Event() {
  count2++;
  /*Serial.println("motor2 event");
  if (digitalRead(Motor2In1) == HIGH) {
    if (digitalRead(Motor2In2) == LOW) {
      count2++;
    }
    else {
      count2--;
    }
  }
  else {
    if (digitalRead(Motor2In2) == LOW) {
      count2--;
    }
    else {
      count2++;
    }
  }*/
}
