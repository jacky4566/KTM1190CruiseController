static int resolution = 12;

void setup() {
  pinMode(8, OUTPUT);
  analogWrite(PIN_DAC0, i);
  analogWriteResolution(resolution);
}

void loop() {
  analogWrite(PIN_DAC0, i);
  analogWrite(PIN_DAC1, i);
  i++;
  if (i >= pow(2, resolution)) {
    i = 0;
  }
}
