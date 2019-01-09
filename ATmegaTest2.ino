#include <SoftwareSerial.h>

int busy = 7;                     // BUSY signal pin

SoftwareSerial mySerial(10, 11);  // RX = 10, TX = 11

void setup() {
  pinMode(busy, OUTPUT);          // Set BUSY pin as output

  mySerial.begin(4800);           // MSP430 baud rate
  Serial.begin(4800);             // PC read data baud rate
}

void loop() {
  digitalWrite(busy, HIGH);       // Busy, dont send data

  delay(100);                     // 10 second delay

  digitalWrite(busy, LOW);        // Not Busy, send data

  while (mySerial.available()) {
    Serial.print((mySerial.read()));   // Print data to serial monitor
    
    digitalWrite(busy, HIGH);     // Busy, dont send data
    break;
  }
}
