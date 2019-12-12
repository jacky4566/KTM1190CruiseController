//Control scheme for Veridian Cruise Control
//Applicable Model: KTM 1190
//All units metric where applicable

#include <STM32L4_CAN.h>

STM32L4_CAN cancan;

CAN_msg_t derpTX{
  0x16,
  {0x080, 0x080, 0x080, 0x080, 0x080, 0x080, 0x08, 0x080},
  0x08
};

void setup() {
  Serial.begin(9600);
}

void loop() {
  cancan.CANSend(&derpTX);
  if (cancan.CANMsgAvail()) {
    CAN_msg_t derpRX;
    cancan.CANReceive(&derpRX);
    Serial.print("Message! ");
    Serial.println(derpRX.data[2]);
  }
  Serial.print("MCR ");
  Serial.println(CAN1->MCR, BIN);
  Serial.print("MSR ");
  Serial.println(CAN1->MSR, BIN);
  Serial.print("ESR ");
  Serial.println(CAN1->ESR, BIN);
  delay(2000);
}
