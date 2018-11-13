#include <HardwareSerial.h>

#define LED           32U
#define BOOT          18U
#define RST           19U

// Client ESP32
HardwareSerial sparthan(2);           // Needed since this is an ESP32

void setup() {
    
  pinMode(BOOT, OUTPUT);
  pinMode(LED, OUTPUT);
  pinMode(RST, OUTPUT);

  digitalWrite(BOOT, LOW);
  digitalWrite(RST, LOW);
  delay(100);
  pinMode(RST, INPUT);
  delay(100);
  digitalWrite(LED, HIGH);
  
  Serial.begin(115200);
  sparthan.begin(3000000, SERIAL_8N1, 16, 17);
}

void loop() {  
  if (Serial.available())             // If data is sent to th se ESP32
  {      
    int c = Serial.read();
    sparthan.write(c);                // Push it through Port 2 to STM32
  }
  if (sparthan.available())           // If data is received through Port 2 from STM32
  {     
    int c = sparthan.read();
    Serial.write(c);                  // Show data on Serial Monitor
  }
}
