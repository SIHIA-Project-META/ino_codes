#define FLASH 4

void setup() {
  pinMode(FLASH, OUTPUT);

}

void loop() {
  digitalWrite(FLASH, HIGH);
  delay(1000);
  digitalWrite(FLASH, LOW);
  delay(1000);
}
