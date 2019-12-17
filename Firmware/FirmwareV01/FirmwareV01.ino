#include <PID_v1.h>
#include <Bounce2.h>
#include <EEPROM.h>
#include <STM32L4_CAN.h>

//Pins
const int pinAnalogA = 0;
const int pinAnalogB = 1;
const int pinOutputA = PIN_DAC0;
const int pinOutputB = PIN_DAC1;
const int pinRedLED = 30;
const int pinGreenLED = 10;
const int pinBlueLED = 8;
const int pinButtonUP = 23;
const int pinButtonDOWN = 24;
const int pinButtonMID = 22;

//Buttons
Bounce debouncerUP = Bounce();
Bounce debouncerDOWN = Bounce();
Bounce debouncerMID = Bounce();

//Can
STM32L4_CAN cancan;
//CAN information
uint32_t lastCANtime;
uint16_t canRPM;
uint16_t canFBRAKE;
uint16_t canFWHEEL;
uint16_t canRWHEEL;
uint8_t canThrottle;
uint8_t canGEAR;
uint8_t canBRAKELIGHT;
uint8_t canECUIntervention;

//Globals
int error = 0;

//PID
double PIDwheelSpeed, PIDoutputThrottle, PIDcruiseSetSpeed;
double Kp = 5, Ki = 1, Kd = 0; //Cruise PID
PID cruisePID(&wheelSpeed, &outputThrottle, &cruiseSetSpeed, Kp, Ki, Kd, DIRECT);

//EEPROM
struct configurationFile {
  uint16_t TPSAMAX;
  uint16_t TPSAMIN;
  uint16_t TPSBMAX;
  uint16_t TPSBMIN;
  uint16_t ConfigDone;
} currentConfig;

void setup() {
  Serial.begin(9600);

  //Pin IO
  debouncerUP.attach(pinButtonUP);
  debouncerUP.interval(6);
  debouncerDOWN.attach(pinButtonDOWN);
  debouncerDOWN.interval(6);
  debouncerMID.attach(pinButtonMID);
  debouncerMID.interval(6);
  analogReadPeriod(2); //2 is default, can be lengthened for smoothing
  analogReadResolution(12);
  analogWriteResolution(12);

  //Math
  cruisePID.SetMode(AUTOMATIC);
  cruisePID.SetSampleTime(10);

  //check for setup mode
  debouncerMID.update();
  delay(5);
  debouncerMID.update();
  if (debouncerMID.read() == LOW) {
    while (debouncerMID.read() == LOW && debouncerMID.duration() < 5000) {
      LEDupdate(setupTimer % 0x3ff, 0, 0); //pulse red
      debouncerMID.update();
      delayMicroseconds(10);
    }
    if (debouncerMID.duration() >= 5000) { //pin was held long enough for setup mode
      setupMode();
    }
  }

  //EEPROM
  configINIT();
}

void loop() {
  //Inputs
  CANprocessor();
  readIO();
  //process information
  Cruiseprocessor();
  //Outputs
  calcOutputs();
  //Teritary User Interface
  UIprocessor();
}

void Cruiseprocessor() {
  static boolean cruiseEN = false;
  if (sanityChecker()) { //Check all the new information is good.
	PIDwheelSpeed = (double)canRWHEEL;
	cruisePID.Compute();
  } else { //something is not right, disable cruise
    cruiseEN = false;
  }
}

boolean sanityChecker() {
  if (canECUIntervention) { //Some ecu intervention took place
    canECUIntervention = 0;
    return 1;
  }
}

void configINIT() {
  configurationFile storedConfig;
  EEPROM.get(0, storedConfig);
  if (storedConfig.ConfigDone !=  0x0f7f) { //no configuration set
    while (1) {
      LEDupdate(0x3FF, 0, 0);
      delay(500);
      LEDupdate(0, 0, 0);
      delay(500);
    }
  } else {
    currentConfig = storedConfig; //good to go
  }
}

void LEDupdate(uint32_t R, uint32_t G, uint32_t B) {
  analogWrite(pinRedLED, R % 0x3FF);
  analogWrite(pinGreenLED, G % 0x3FF);
  analogWrite(pinBlueLED, B & 0x3FF);
}


void readIO() { //Read all the appropriate signals, Check for errors
  debouncerUP.update();
  debouncerMID.update();
  debouncerDOWN.update();
  inputA = analogRead(pinAnalogA);
  inputB = analogRead(pinAnalogB);
}

void calcOutputs() {
  analogWrite(pinOutputA, outputA);
  analogWrite(pinOutputB, outputB);
}

void UIprocessor() {
  /* ERROR MESSAGES
    1 Input A too low
    2 Input B too low
    3 Input A too high
    4 Input B too high
    5 NO CAN
  */
  static uint32_t lastPrint = 0;
  if (lastPrint  + 2000 <= millis() && !error) {
    switch (error) {
      case 0:
        if (cruiseEnabled) {
          RGBwriter(0, 0, 4095); //Blue
        } else {
          RGBwriter(0, 4095, 0); //Green
        }
        break;
      case 1:
        LEDupdate(4095, 0, 0); // Red
        Serial.println("1 Input A too low");
        break;
      case 2:
        LEDupdate(4095, 0, 0); // Red
        Serial.println("2 Input B too low");
        break;
      case 3:
        LEDupdate(4095, 0, 0); // Red
        Serial.println("3 Input A too high");
        break;
      case 4:
        LEDupdate(4095, 0, 0); // Red
        Serial.println("4 Input B too high");
        break;
      case 5:
        LEDupdate(4095, 0, 0); // Red
        Serial.println("5 NO CAN");
        break;
    }
    lastPrint = millis();
  }
}

void LEDupdate(uint32_t R, uint32_t G, uint32_t B) {
  analogWrite(pinRedLED, R % 0x3FF);
  analogWrite(pinGreenLED, G % 0x3FF);
  analogWrite(pinBlueLED, B & 0x3FF);
}

void CANprocessor() {
  static uint32_t lastCANtime;
  while (cancan.CANMsgAvail()) {
    CAN_msg_t canRXmsg;
    cancan.CANReceive(&canRXmsg);
    switch (canRXmsg.id) {
      case 0x120: //RPM
        if (canRXmsg.len == 8) {
          canRPM = (canRXmsg.data[0] << 8) | canRXmsg.data[1];
          canThrottle = canRXmsg.data[2];
          lastCANtime = millis();
        }
        break;
      case 0x129: //Gear
        if (canRXmsg.len == 8) {
          canGEAR = canRXmsg.data[0];
          lastCANtime = millis();
        }
        break;
      case 0x290: //Front Brake
        if (canRXmsg.len == 8) {
          canFBRAKE = (canRXmsg.data[0] << 8) | canRXmsg.data[1];
          lastCANtime = millis();
        }
        break;
      case 0x12b: //Wheel Speeds
        if (canRXmsg.len == 8) {
          canFWHEEL = (canRXmsg.data[0] << 8) | canRXmsg.data[1];
          canRWHEEL = (canRXmsg.data[2] << 8) | canRXmsg.data[3];
          lastCANtime = millis();
        }
        break;
      case 0x450: //ABS MTC MODE change or intervention
        if (canRXmsg.len == 8) {
          byte interventionAdder;
          for (int i = 0; i <= 7; i++) {
            interventionAdder &= canRXmsg.data[i];
          }
          if (interventionAdder != 0) {
            canECUIntervention = interventionAdder;
          }
          lastCANtime = millis();
        }
        break;
      case 0x550: //Brake light
        if (canRXmsg.len == 8) {
          canBRAKELIGHT = canRXmsg.data[5];
          lastCANtime = millis();
        }
        break;
    }
  }
  if ((lastCANtime + 1000) <= millis()) {
    error = 5;
  }
}
