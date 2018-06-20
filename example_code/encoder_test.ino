//define pins
//an encoder generally has two hall effect sensors
//one is sufficient for measuring speed only, but two are necessary to also know the direction of rotation
//EncoderA and EncoderB are the inputs from the two hall effect sensors on the encoder
const int EncoderA = 2;
const int EncoderB = 3;
//the motor has two outputs: to make it turn one is set to HIGH and one to LOW
//switching which is HIGH will change the direction of rotation.
const int MotorOut1 = 8;
const int MotorOut2 = 9;

//other constants
const int delayTime = 1000; //delay at the end of each loop before repeating
const int eventsPerRev = 228; //number of events per rotation of the motor shaft that the encoder can count

//define counters
volatile double encoderCount = 0;
volatile double loopCount = 0;
volatile float rotations = 0;

void setup()
{
  //set pin modes for motor pins
  pinMode(EncoderA, INPUT);
  pinMode(EncoderB, INPUT);
  pinMode(MotorOut1, OUTPUT);
  pinMode(MotorOut2, OUTPUT);
  digitalWrite(EncoderA, HIGH); //this enables the pull-up resistor on the EncoderA pin, this is often necessary with encoders
  digitalWrite(EncoderB, HIGH); //same for EncoderB pin

  //initialize hardware interrupts
  //this line tells the Arduino to call a specified function anytime a certain event occurs on the specified interrupt pin
  //the first argument sent to this function is the pin to watch for interrupt events, but the numbering system for interrupt pins is NOT the same as the one used to define the pins above
  //interrupt pins have their own numbering system starting from 0, and going to 1 since there's only two of them on a typical Arduino
  //pin 2 is the first interrupt pin, so interrupt pin 0 corresponds to pin #2 and interrupt pin 1 corresponds to pin #3
  //you can get around this, as I have here, by using the normal pin numbers with the digitalPinToInterrupt() function
  //the second argument is the name of the function, EncoderEvent in this case, to be called when an event is detected on the interrupt pin
  //the third argument defines what type of events watch for, and is either RISING, FALLING, or CHANGE
  //RISING will call the specified function anytime the specified pin rises from LOW to HIGH
  //FALLING will do the opposite: only call the function when the pin falls from HIGH to LOW
  //CHANGE will call the function on both rising and falling events
  attachInterrupt(digitalPinToInterrupt(EncoderA), EncoderEvent, CHANGE); //pin 2/interrupt pin 0
  
  //initialize serial communication
  //this lets you output information to the serial monitor in the Arduino software
  Serial.flush();
  Serial.begin(9600);
  
  //start motors
  //in this example the motor is started once here and just spins forever to demonstrate the encoder code in the loop below
  digitalWrite(MotorOut1, HIGH);
  digitalWrite(MotorOut2, LOW);
}

void loop()
{
  //display elapsed time in the serial monitor
  Serial.print("Time: ");
  Serial.print(loopCount * delayTime / 1000);
  Serial.println(" s");
  
  //calculate motor rotations since last loop and add it to the total number of rotations
  //encoderCount is how many HIGH signals the encoder has sent since the last loop
  //eventsPerRev is the number of countable events per one rotation of the motor shaft
  //eventsPerRev is multiplied by two because the interrupt is called on CHANGE as defined above
  //this means for every HIGH signal sent by the encoder the Arduino will count twice: once on the LOW to HIGH rise, and once on the HIGH to LOW fall
  //to correct for this double counting eventsPerRev is multiplied by two, or you could set the interrupt function to call on RISING or FALLING instead of CHANGE
  rotations += float(encoderCount) / (eventsPerRev * 2);
  
  //encoderCount is reset to zero every loop to prevent it from overflowing
  //it is very easy to overflow the variable with very strange effects if you don't occasionally reset the counter
  encoderCount = 0;
  
  //display total motor rotations in the serial monitor
  Serial.print("Motor rotations: ");
  Serial.println(rotations);
  
  //increment loop counter and delay until next loop iteration
  loopCount++;
  delay(delayTime);
}

//encoder event function
//this is called whenever there is a RISING/FALLING/CHANGE event (see above) on the EncoderA pin
//two variations are shown here: simply using encoderCount++ is sufficient to measure speed or rotation angle if you know what direction the motor is turning
//the commented code can be used if the motor sometimes changes directions and you need to know how far it's rotated versus its starting position
//in the latter case the counter will go up with rotation in one direction and go down with rotation in the other, so it could be positive or negative depending
//on its current position versus where it started, and the sign will indicate which direction it's rotated
void EncoderEvent() {
  encoderCount++;
  
  /*if (digitalRead(EncoderA) == HIGH) {
    if (digitalRead(EncoderB) == LOW) {
      encoderCount++;
    }
    else {
      encoderCount--;
    }
  }
  else {
    if (digitalRead(EncoderB) == LOW) {
      encoderCount--;
    }
    else {
      encoderCount++;
    }
  }*/
}