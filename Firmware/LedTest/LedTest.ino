#include <elapsedMillis.h>

const int pinRedLED = 30;
const int pinGreenLED = 10;
const int pinBlueLED = 5;

void setup() {
  analogWriteResolution(12);
  pinMode(pinRedLED, OUTPUT);
  pinMode(pinGreenLED, OUTPUT);
  pinMode(pinBlueLED, OUTPUT);

}

void loop() {

  digitalWrite(pinRedLED, HIGH);
  delay(1000);
  digitalWrite(pinRedLED, LOW);
  digitalWrite(pinGreenLED, HIGH);
  delay(1000);
  digitalWrite(pinGreenLED, LOW);
  digitalWrite(pinBlueLED, HIGH);
  delay(1000);
  digitalWrite(pinBlueLED, LOW);
}

void LEDupdate(uint32_t R, uint32_t G, uint32_t B) {
  analogWrite(pinRedLED, R % 0x3FF);
  analogWrite(pinGreenLED, G % 0x3FF);
  analogWrite(pinBlueLED, B & 0x3FF);
}
