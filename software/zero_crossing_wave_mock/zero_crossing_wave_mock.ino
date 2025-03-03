const int pin = 10;
const float frequency = 0.25;
const float period = 1000 / frequency; // period in milliseconds

void setup() {
  pinMode(pin, OUTPUT);
}

void loop() {
  digitalWrite(pin, HIGH);
  delay(period / 2); // half period
  digitalWrite(pin, LOW);
  delay(period / 2); // half period
}