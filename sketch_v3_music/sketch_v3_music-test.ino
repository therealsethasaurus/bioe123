// This code is test code for the BioE 123 Centrifuge V3
//
// For the most part, this code is the same as the V2 centrifuge code.
// However, now it starts by checking if it received an input from Python
// to signal that everything has started before prompting the user to enter input.
//
//  Seth Kohno and Kasey Lassen; 20260312

// define constants and pins
int motorGatePin = 5;

// define variables
// analog input + pwm variables
unsigned int dutyCycle = 0;  // duty cycle to send to mosfet to run motor (PWM)
int dutyCycleSS = 100;

long currRPM;

// parameters to be specified by the user
int desiredSpeed;
unsigned long desiredDuration;

unsigned long startTime = 0;
unsigned long prevTime;
unsigned long prevDisplayUpdate = 0;

// parameters for the PID controller
float Kp = 0.3;
float Ki = 0;
float Kd = 0;

// running values for the PID controller
int prevError = 0;
int integral = 0;

bool spinning = false;  // indicator for the state of the centrifuge

int stop_speed = 100;

// declare intercept pin - this is pin d2, not the pd2 [int 2] one!
const byte inputPin = 2;  // intercept pin

// count the number of half spins
volatile unsigned long halfSpinCount = 0;
unsigned long prevCount = 0;

unsigned long prevMillis = 0;

/**
Record when the button is pressed
*/
void signalChange() {
  ++halfSpinCount;
}

/**
* Use PID controller to determine how much to change the speed
*/
int pidController(int currentSpeed, int desiredSpeed, int dt) {
  int e = desiredSpeed - currentSpeed;
  integral = integral + e * dt;
  int d = (e - prevError) / dt;
  int out = Kp * e + Ki * integral + Kd * d;
  prevError = e;

  return out;
}


void setup() {
  attachInterrupt(digitalPinToInterrupt(inputPin), signalChange, RISING);
  analogWrite(motorGatePin, 0);

  Serial.begin(9600);

  while (!Serial)
    ;  // wait for serial monitor to finish loading

  
  while (Serial.available() == 0);

  // confirm connection with pyserial
  char c = Serial.parseInt();
  if (c != 1) {
    Serial.println("Aborting. Incorrect signal - cannot confirm connection with Pyserial.");
    exit(1);
  }
  Serial.println("Successfully connected to the code");

  Serial.println("Please enter desired motor speed (in RPM) ");

  while (Serial.available() == 0);

  desiredSpeed = Serial.parseInt();
  Serial.println((String) "desired speed is " + desiredSpeed);

  delay(1000);

  Serial.println("Please enter desired duration (in seconds) (hint 60secs=1min): ");
  while (Serial.available() == 0);

  desiredDuration = Serial.parseInt();
  Serial.println((String) "desired duration is " + desiredDuration);
  
  delay(1000);

  Serial.println("Start spinning? (enter y/n)");
  delay(1000);

  while (Serial.read() != 'y') {
    // tell the user that they messed up :)
    Serial.println("try again :(");
    delay(1000);
  }
  Serial.println("Starting to spin");
  digitalWrite(motorGatePin, 100);
  spinning = true;
}

void loop() {
  if (spinning) {
    // set the start time
    if (startTime == 0) {  // used to be -1, but unsignged, and assumed edge case of millis being 0 is so rare
      startTime = millis();
      prevDisplayUpdate = startTime;
    }

    // calcuate change in spin counts since last iteration
    unsigned long spinCountDelta = halfSpinCount - prevCount;
    unsigned long currTime = millis();

    // if spin count has changed, recalculate the speed and control values
    // experimented with other values here in V2, but for V3, 0 seemed to work well enough :)
    if (spinCountDelta > 0) {
      // calculate the centrfiuges speed
      unsigned long freq = (1000 * spinCountDelta) / (currTime - prevMillis);
      freq = freq / 2;
      currRPM = freq * 60;
      Serial.println((String) "Current RPM" + currRPM);

      // update the dutycycle
      int dt = currTime - prevMillis;
      int control = pidController(currRPM, desiredSpeed, dt);

      if (control > 155) {
        dutyCycle = 255;
      } else if (control < -100 ) {
        dutyCycle = 0;
      } else {
        dutyCycle = control + dutyCycleSS;
      }

      analogWrite(motorGatePin, dutyCycle);

      // update the clock
      prevMillis = currTime;
      prevCount = halfSpinCount;
    }

    // if time is up, stop the centrifuge
    if ((currTime - startTime) > desiredDuration * 1000) {
      digitalWrite(motorGatePin, 0);
      spinning = false;
      Serial.println("Centrifuge is slowing down");
    }

    // check if the user has sent a stop signal to the Arduino
    if (Serial.available() > 0) {
      char userInput = Serial.read();

      // if user types in an 's' while the centrifuge is spinning, emergency stop!
      if (userInput == 's') {
        spinning = false;
        digitalWrite(motorGatePin, 0);
        Serial.println("Emergency stopping procedure has been activated");
      }
    }
  } else {
    // centrifuge should not be spinning -> dutyCycle = 0;
    digitalWrite(motorGatePin, 0);
    unsigned long currTime = millis();

    if (currRPM < stop_speed) {
      Serial.println("Centrifuge is done spinning - you may collect your samples now!");
      delay(1000);
    } else {
      if (currTime - prevMillis > 1000) {
        // still need to update the speeds when it's not spinning
        unsigned long spinCountDelta = halfSpinCount - prevCount;
        currTime = millis();

        // calculate the centrfiuges speed
        unsigned long freq = (1000 * spinCountDelta) / (currTime - prevMillis);
        freq = freq;
        currRPM = freq * 60;

        // update the clock
        prevMillis = currTime;
        prevCount = halfSpinCount;

        Serial.println("Protocol over: Current RPM Do not remove samples yet");
      }
    }
  }
}