#include <STM32L4_CAN.h>

STM32L4_CAN CanCan(CAN_500KBPS);

void setup() {
  Serial.begin(9600);
}

void loop() {
  while (CanCan.CANMsgAvail()){
    CAN_msg_t newMessage;
    CanCan.CANReceive(&newMessage);
    Serial.print("New CanCan from: 0x");
    Serial.println(newMessage.id,HEX);
    Serial.print("DATA: ");
    for (int i = 0 ; i<8; i++){
      Serial.print(" 0x");
      Serial.print(newMessage.data[i], HEX);
    }
  }
}