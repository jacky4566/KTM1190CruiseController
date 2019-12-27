#include <PID_v1.h>
#include <Bounce2.h>
#include <EEPROM.h>
#include <STM32L4_CAN.h>

//#define channelAinveted
//#define channelBinveted

//Pins
const int pinAnalogA = 0;
const int pinAnalogB = 1;
const int pinOutputA = PIN_DAC0;
const int pinOutputB = PIN_DAC1;
const int pinInterSW = PIN_DAC0;
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
const int debounceTime = 5;  //ms

//IO globals
const double inputTolerance = 0.05; //5% tolerance on adc inputs
double handThrottlePOS;
double outputThrottle;

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
boolean cruiseEN = false;
double cruiseSetSpeed = 0;

//PID
double PIDwheelSpeed, PIDoutputThrottle, PIDcruiseSetSpeed;
double Kp = 5, Ki = 1, Kd = 0; //Cruise PID
PID cruisePID(&wheelSpeed, &outputThrottle, &cruiseSetSpeed, Kp, Ki, Kd, DIRECT);

//EEPROM
struct configurationFile {
  double TPSAMAX = 0;
  double TPSAMIN = 0;
  double TPSARANGE = 0;
  double TPSBMAX = 0;
  double TPSBMIN = 0;
  double TPSBRANGE = 0;
  uint16_t ConfigDone; //should read 0x0f7f
} currentConfig;

void setup() {
  Serial.begin(9600);

  //Pin IO
  debouncerUP.attach(pinButtonUP);
  debouncerUP.interval(debounceTime);
  debouncerDOWN.attach(pinButtonDOWN);
  debouncerDOWN.interval(debounceTime);
  debouncerMID.attach(pinButtonMID);
  debouncerMID.interval(debounceTime);
  analogReadPeriod(4); 		//2 is default, can be lengthened up to 7 for smoothing
  analogReadResolution(12);	//use full 12bit mode
  analogWriteResolution(12);//use full 12bit mode

  //PID
  cruisePID.SetMode(AUTOMATIC);
  cruisePID.SetSampleTime(10);

  //check for setup mode
  debouncerUP.update();
  delay(5);
  debouncerUP.update();
  if (debouncerUP.read() == LOW) {
    while (debouncerUP.read() == LOW && debouncerUP.duration() < 5000) {
      LEDupdate(setupTimer % 0x3ff, 0, 0); //pulse red
      debouncerUP.update();
      delayMicroseconds(10);
    }
    if (debouncerUP.duration() >= 5000) { //pin was held long enough for setup mode
      setupMode();
    }
  }

  //EEPROM
  configINIT();
}

void loop() {
  //Inputs
  error = CANprocessor();
  error = readIO();
  //Outputs
  error = calcOutputs();
  //Teritary User Interface
  UIprocessor();
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


byte readIO() { //Read all the appropriate signals, Check for errors
	//BUTTONS
	debouncerUP.update();
	debouncerMID.update();
	debouncerDOWN.update();
	
	//button processing
	if (short press up){
	}
	else if (long press up){
	}
	else if (short press down){
	}
	else if (long press down){
	}
	
	//TPS
	double ADCinputA = (double)analogRead(pinAnalogA);
	double ADCinputB = (double)analogRead(pinAnalogB);
	if (ADCinputA  <= currentConfig.TPSAMIN * (1 - inputTolerance)){
		return 1;
	} else if(ADCinputB  <= currentConfig.TPSBMIN * (1 - inputTolerance)){
		return 2;
	}else if(ADCinputA  >= currentConfig.TPSAMAX * (1 + inputTolerance)){
		return 3;
	}else if(ADCinputB  <= currentConfig.TPSBMAX * (1 + inputTolerance)){
		return 4;
	}else{
		handThrottlePOS = ((ADCinputA - currentConfig.TPSAMIN)/(currentConfig.TPSARANGE) + (ADCinputB - currentConfig.TPSBMIN)/(currentConfig.TPSBRANGE)) / 2;
	}
	return 0;
}

void calcOutputs() {
	if (cruiseEN && error == 0){
		digitalWrite(pinInterSW, HIGH);
		PIDwheelSpeed = (double)canRWHEEL;
		cruisePID.Compute();
		analogWrite(pinOutputA, (outputThrottle * currentConfig.TPSARANGE) + currentConfig.TPSAMIN);
		analogWrite(pinOutputB, (outputThrottle * currentConfig.TPSBRANGE) + currentConfig.TPSBMIN);
	}else{
		digitalWrite(pinInterSW, LOW);
		STM32.Sleep();//cruise isnt doing much of anything here. 
	}
	
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

void setupMode(){
	configurationFile newConfig;
	newConfig.TPSAMAX = channelA;
	newConfig.TPSAMIN = channelA;
	newConfig.TPSBMAX = channelB;
	newConfig.TPSBMIN = channelB;
	while(debouncerUP.read() == LOW){
		LEDupdate(millis() % 0x3ff, 0, 0); //pulse red
		channelA = analogRead(pinAnalogA);
		channelB = analogRead(pinAnalogB);
		debouncerUP.update();
		//Get the boundary conditions as users twists throttle
		if (channelA < newConfig.TPSAMIN){
			newConfig.TPSAMIN = channelA;
		}
		if (channelA > newConfig.TPSAMAX){
			newConfig.TPSAMAX = channelA;
		}
		if (channelB < newConfig.TPSBMIN){
			newConfig.TPSAMIN = channelB;
		}
		if (channelB < newConfig.TPSBMIN){
			newConfig.TPSAMIN = channelB;
		}
	}
	newConfig.TPSARANGE = (newConfig.TPSAMAX - newConfig.TPSAMIN);
	newConfig.TPSBRANGE = (newConfig.TPSBMAX - newConfig.TPSBMIN);
	newConfig.ConfigDone = 0x0f7f;
	if (STM32.getVREF() >= 2.0){ //check for good power supply
		EEPROM.put(0, newConfig); //write to memory
	}
	currentConfig = newConfig;
}

byte CANprocessor() {
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
    return 5;
  }else {
	  return 0;
  }
}
