// This code is test code for the BioE 123 Centrifuge V2 Prototype
//
// Key elements:
//
//
//  Seth Kohno and Kasey Lassen; 20260224

// define constants and pins
int motorGatePin = 5;

// define variables
// analog input + pwm variables
unsigned int dutyCycle = 0;  // duty cycle to send to mosfet to run motor (PWM)
unsigned int dutyCycleSS = 100;

long currRPM;

// parameters to be specified by the user
float desiredSpeed = 2500;    // default is 2500 rpm
float desiredDuration = 120;  // default is 2 minutes (120 seconds)

unsigned long startTime = 0;
unsigned long prevTime;
unsigned long prevDisplayUpdate = 0;

// parameters for the PID controller
// we may choose to not use all 3 components,
// and if so, we will just set those parameters to 0
int Kp = 1;
int Ki = 0;
int Kd = 0;

// running values for the PID controller
int prevError = 0;
int integral = 0;

bool spinning = false;  // indicator for the state of the centrifuge

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

  Serial.println("Please enter desired motor speed (in RPM) ");
  while (Serial.available() == 0) {
    // do nothing
  }
  desiredSpeed = Serial.parseInt();
  Serial.println((String) "desired speed is " + desiredSpeed);

  delay(1000);

  Serial.println("Please enter desired duration (in seconds) (hint 60secs=1min): ");
  delay(1000);
  while (Serial.available() == 0) {
    // do nothing until use enters duration
  }
  desiredDuration = Serial.parseInt();
  Serial.println((String) "desired duration is " + desiredDuration);

  Serial.println("Start spinning? (y/n)");
  delay(1000);

  while (Serial.read() != 'y') {
    Serial.println("try again :(");
    delay(1000);
  }
  Serial.println("Starting to spin");
  digitalWrite(motorGatePin, 100);
  spinning = true;
}

void loop() {
  // put your main code here, to run repeatedly:
  if (spinning) {
    // Serial.println("Centrifuge is now spinning");
    // set the start time
    if (startTime == 0) {  // used to be -1, but this would be weird with unsigned type
      startTime = millis();
      prevDisplayUpdate = startTime;
    }

    // calcuate change in spin counts since last iteration
    unsigned long spinCountDelta = halfSpinCount - prevCount;
    // Serial.println((String) "count " + halfSpinCount);
    unsigned long currTime = millis();

    // Serial.println((String) "halfSpin count" + halfSpinCount);
    // if spin count has changed, recalculate the speed and control values
    if (spinCountDelta > 0) {
    // if (currTime - prevMillis > 20) {
      // calculate the centrfiuges speed
      unsigned long freq = (1000 * spinCountDelta) / (currTime - prevMillis);
      freq = freq / 2;
      currRPM = freq * 60;
      Serial.println((String) "Current RPM" + currRPM);

      // update the dutycycle
      int dt = currTime - prevMillis;
      int control = pidController(currRPM, desiredSpeed, dt);

      // Serial.println((String) "Value from controller" + control);
      // dutyCycle = control / 2 + dutyCycleSS;  // add the dutyCycleSS as a baseline guess for what the duty cycle will be
      // dutyCycle = min(dutyCycle, 255);
      // dutyCycle = max(0, dutyCycle);
      Serial.println((String) "control " + control);

      if (control / 2 > 155) {
        dutyCycle = 255;
      } else if (control / 2 < -100 ) {
        dutyCycle = 0;
      } else {
        dutyCycle = control / 2 + dutyCycleSS;
      }

      // int dutyCycleInt = control / 2 + dutyCycle;
      // dutyCycleInt = min(dutyCycle, 255);
      // dutyCycleInt = max(0, dutyCycle);
      // dutyCycle = dutyCycleInt;
      analogWrite(motorGatePin, dutyCycle);

      // update the clock
      prevMillis = currTime;
      prevCount = halfSpinCount;
    }

    // if time is up, stop the centrifuge
    if ((currTime - startTime) > desiredDuration * 1000 + 2000) {
      analogWrite(motorGatePin, 0);
      spinning = false;
      Serial.println("Centrifuge is slowing down");
    }

    // check if the user has sent a stop signal to the Arduino
    if (Serial.available() > 0) {
      char userInput = Serial.read();

      // if user types in an 's' while the centrifuge is spinning, emergency stop!
      if (userInput == 's') {
        spinning = false;
        analogWrite(motorGatePin, 0);
        Serial.println("Emergency stopping procedure has been activated");
      }
    }
  } else {
    // centrifuge should not be spinning -> dutyCycle = 0;
    analogWrite(motorGatePin, 0);
    unsigned long currTime = millis();

    if (currRPM < 10000) {
      Serial.println("Centrifuge is done spinning - you may collect your samples now!");
      delay(1000);
    } else {
      if (currTime - prevMillis > 1000) {
        // still need to update the speeds when it's not spinning
        unsigned long spinCountDelta = halfSpinCount - prevCount;
        currTime = millis();

        // calculate the centrfiuges speed
        unsigned long freq = (1000 * spinCountDelta) / (currTime - prevMillis);
        Serial.println(currTime - prevMillis);
        freq = freq;
        currRPM = freq * 60;

        // update the clock
        prevMillis = currTime;
        prevCount = halfSpinCount;

        Serial.println((String) "Protocol over: Current RPM" + currRPM + " Do not remove samples yet");
      }
    }
  }
}
