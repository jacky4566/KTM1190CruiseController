#include <Bounce2.h>
#include <elapsedMillis.h>

const int pinRedLED = 30;
const int pinGreenLED = 10;
const int pinBlueLED = 8;
const int pinButtonUP = 23;
const int pinButtonDOWN = 24;
const int pinButtonMID = 22;

Bounce debouncerUP = Bounce();
Bounce debouncerDOWN = Bounce();
Bounce debouncerMID = Bounce();

void setup() {
  Serial.begin(9600);
  debouncerUP.attach(pinButtonUP);
  debouncerUP.interval(5);
  debouncerDOWN.attach(pinButtonDOWN);
  debouncerDOWN.interval(5);
  debouncerMID.attach(pinButtonMID);
  debouncerMID.interval(5);

  debouncerMID.update();
  if (debouncerMID.read() == LOW) {
    elapsedMillis setupTimer;
    while (debouncerMID.read() == LOW && setupTimer < 5000) {
      LEDupdate(setupTimer % 0x3ff, 0, 0); //pulse red
      debouncerMID.update();
      delayMicroseconds(10);
    }
    if (debouncerMID.duration() > 5000) { //pin was held long enough for setup mode
      LEDupdate(0, 1024, 0);
      while (1) {};
    }
  } else {
    LEDupdate(0, 4095, 4095);
  }

}

void loop() {
  debouncerMID.update();
  Serial.println(debouncerMID.read());

  delay(1000);
}

void LEDupdate(uint32_t R, uint32_t G, uint32_t B) {
  analogWrite(pinRedLED, R % 0x3FF);
  analogWrite(pinGreenLED, G % 0x3FF);
  analogWrite(pinBlueLED, B & 0x3FF);
}
