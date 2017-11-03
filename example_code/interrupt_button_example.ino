//this file demonstrates use of the interrupt pins
//when the input signal to an interrupt pin changes it immediately calls some function as specified in setup() by the attachInterrupt() function
//there are three options for the third argument of attachInterrupt: CHANGE, RISING, and FALLING
//CHANGE calls the interrupt function everytime the input to the interrupt pin is changed, either HIGH to LOW -or- LOW to HIGH
//FALLING only calls the interrupt function when the interrupt pin value changes from HIGH to LOW
//RISING only calls the interrupt function when the interrupt pin value changes from LOW to HIGH

const int Button1 = 3;
const int LED = 9;

void setup()
{
  pinMode(Button1, INPUT);
  pinMode(LED, OUTPUT);
  
  //attach interrupt for button to trigger on changed state
  attachInterrupt(digitalPinToInterrupt(Button1), ButtonEvent, RISING); //pin 3

  Serial.begin(9600);
}

void loop()
{
  delay(100);
}

void ButtonEvent() {
  Serial.println("button pressed");
  digitalWrite(LED, HIGH);
  delay(1000);
  digitalWrite(LED, LOW);
}