//Control scheme for Veridian Cruise Control
//Applicable Model: KTM 1190
//All units metric where applicable

#include <STM32L4_CAN.h>

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

void setup() {
  Serial.begin(9600);
}

void loop() {
  CANprocessor();
  if (millis() % 2000 == 0) {
    if ((lastCANtime + 1000) <= millis()) {
      Serial.println("NO CAN");
    } else {
      Serial.print("RPM: ");
      Serial.println(canRPM);
      Serial.print("canFBRAKE: ");
      Serial.println(canFBRAKE);
      Serial.print("canFWHEEL: ");
      Serial.println(canFWHEEL);
      Serial.print("canRWHEEL: ");
      Serial.println(canRWHEEL);
      Serial.print("canThrottle: ");
      Serial.println(canThrottle);
      Serial.print("canGEAR: ");
      Serial.println(canGEAR);
      Serial.print("canBRAKELIGHT: ");
      Serial.println(canBRAKELIGHT, BIN);
      Serial.print("canECUInter: ");
      Serial.println(canECUIntervention);
      Serial.println();
    }
  }
}

void CANprocessor() {
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
}
