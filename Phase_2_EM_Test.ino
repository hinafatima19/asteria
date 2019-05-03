#include <SoftwareSerial.h>

int BUSY = 7;
int RX = 10;
int TX = 11;

SoftwareSerial mySerial(RX, TX);

void setup() {
  pinMode(BUSY, INPUT);

  mySerial.begin(38400);

  Serial.begin(38400);
}

void loop() {
  while (digitalRead(BUSY) == LOW) {

    Serial.print("\nBUSY IS LOW, TX: ");

    for(int i = 0; i < 3; i++){
      Serial.print(mySerial.write(0x50));
    }

    for (int i = 0; i < 35; i++) {
      Serial.print(mySerial.write(9));
    }

    Serial.print("\nBUSY IS LOW, RX: ");

    while (mySerial.available()) {
      Serial.print(mySerial.read());
    }

    delay(5000);
  }
}
