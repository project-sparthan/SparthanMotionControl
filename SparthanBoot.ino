#define LED           32U
#define BOOT          18U
#define RST           19U

void setup() {
  
  pinMode(BOOT, OUTPUT);
  pinMode(LED, OUTPUT);
  pinMode(RST, OUTPUT);

  digitalWrite(BOOT, LOW);
  digitalWrite(RST, LOW);
  delay(100);
  pinMode(RST, INPUT);

}

void loop() {

  digitalWrite(LED, LOW);
  delay(100);
  digitalWrite(LED, HIGH);
  delay(100);

}
