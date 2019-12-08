#include <PID_v1.h>

//Pins
const int pinAnalogA = 0;
const int pinAnalogB = 1;
const int pinOutputA = PIN_DAC0;
const int pinOutputB = PIN_DAC1;
const int pinRedLED = 3;
const int pinGreenLED = 4;
const int pinBlueLED = 5;

//Tunable variables
const uint32_t maxRPM = 0x5000;
double Kp = 2, Ki = 5, Kd = 1; //Cruise PID

//CAN Codes, to be confirmed
const int canHEXIDRPMandThrottle = 0x120;
const int canHEXIDGear = 0x129; //Check for clutch status here
const int canHEXIDFBrake = 0x290;
const int canHEXIDWheelSpeed = 0x12b;
const int canHEXIDLights = 0x550;
const int canHEXIDNannyIntervention = 0x450;

//Globals
int error = 0;
boolean cruiseEnabled;
double cruiseSetSpeed;
double wheelSpeed;
double inputThrottle;
double outputThrottle;
uint32_t inputA;
uint32_t inputB;
uint32_t outputA;
uint32_t outputB;

//PID
PID cruisePID(&wheelSpeed, &outputThrottle, &cruiseSetSpeed, Kp, Ki, Kd, DIRECT);

void setup() {
  Serial.begin(9600);
  analogReadPeriod(2); //2 is default, can be lengthened for smoothing
  analogReadResolution(12);
  analogWriteResolution(12);
  cruisePID.SetMode(AUTOMATIC);
  cruisePID.SetSampleTime(100);
  cruisePID.SetOutputLimits(40,3000);
}

void loop() {
  inputReads();
  if (cruiseEnabled && (error == 0)) {
    computeCruiseOutputs();
  } else {
    computeNormalOutputs();
  }
  analogWrites();
  errorPrinter();
}

void computeCruiseOutputs() {
  if (inputThrottle > outputThrottle) { //user is accelerating
    computeNormalOutputs();
  }
  else {
    cruisePID.Compute();
    outputA = (uint32_t)outputThrottle;
    outputB = map((uint32_t)outputThrottle, 0, 4095, 4095, 0);
  }
}

void computeNormalOutputs() {
  outputA = inputA;
  outputB = inputB;
}

void inputReads() { //Read all the appropriate signals, Check for errors
  inputA = analogRead(pinAnalogA);
  inputB = analogRead(pinAnalogB);
  if (inputA < minADC) {
    error = 1;
  } else if (inputB < minADC) {
    error = 2;
  }  else if (inputA > maxADC) {
    error = 3;
  }  else if (inputB > maxADC) {
    error = 4;
  }
  inputThrottle = (inputA + map(inputB, 0, 4095, 4095, 0)) / 2; //average the two inputs
}

void analogWrites() {
  analogWrite(pinOutputA, outputA);
  analogWrite(pinOutputB, outputB);
}

void errorPrinter() {
  /* ERROR MESSAGES
    1 Input A too low
    2 Input B too low
    3 Input A too high
    4 Input B too high
  */
  static uint32_t lastPrint = 0;
  if (lastPrint  + 1000 < millis()) {
    switch (error) {
      case 0:
        if (cruiseEnabled) {
          RGBwriter(0, 0, 4095); //Blue
        } else {
          RGBwriter(0, 4095, 0); //Green
        }
        break;
      case 1:
        RGBwriter(4095, 0, 0); // Red
        Serial.println("1 Input A too low");
        break;
      case 2:
        RGBwriter(4095, 0, 0); // Red
        Serial.println("2 Input B too low");
        break;
      case 3:
        RGBwriter(4095, 0, 0); // Red
        Serial.println("3 Input A too high");
        break;
      case 4:
        RGBwriter(4095, 0, 0); // Red
        Serial.println("4 Input B too high");
        break;
    }
    lastPrint = millis();
  }
}

void RGBwriter(int red, int green, int blue) {
  analogWrite(pinRedLED, red);
  analogWrite(pinGreenLED, green);
  analogWrite(pinBlueLED, blue);
}
