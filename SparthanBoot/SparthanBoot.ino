#include <hardwareSerial.h>

#define LED           32U
#define BOOT          18U
#define RST           19U

HardwareSerial STMserial(2);

void setup() {
  
  pinMode(BOOT, OUTPUT);
  pinMode(LED, OUTPUT);
  pinMode(RST, OUTPUT);

  digitalWrite(BOOT, LOW);
  digitalWrite(RST, LOW);
  delay(100);
  pinMode(RST, INPUT);

  STMserial.begin(3000000, SERIAL_8N1, 16, 17);

}

void loop() {

  STMserial.println("TEST SERIAL");
  digitalWrite(LED, LOW);
  delay(100);
  digitalWrite(LED, HIGH);
  delay(100);

}
