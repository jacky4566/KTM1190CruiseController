const int switchPin = 11;
const int pinRedLED = 10;
const int pinGreenLED = 30;
const int pinBlueLED = 5;

uint32_t desiredValue;

void setup() {
  Serial.begin(9600);
  analogWriteResolution(12);
  analogReadResolution(12);
  pinMode(switchPin, OUTPUT);
  pinMode(pinGreenLED, OUTPUT);
  digitalWrite(switchPin, HIGH);
  analogWrite(PIN_DAC0, 0); //use board manager to setup dac
  analogWrite(PIN_DAC1, 0); //use board manager to setup dac
}

void loop() {
  desiredValue = desiredValue + 12;
  analogWrite(PIN_DAC0, desiredValue); //use board manager to setup dac
  analogWrite(PIN_DAC1, desiredValue); //use board manager to setup dac
  analogWrite(pinGreenLED, desiredValue);
  delay(10); //keep things slow
}
