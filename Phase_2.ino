#include <SoftwareSerial.h>

int BUSY = 7;
int RX = 10;
int TX = 11;

int data_frame = 0;

SoftwareSerial mySerial(RX, TX);

void setup() {
  digitalWrite(BUSY, HIGH);       // HIGH = busy, LOW = not busy
  pinMode(BUSY, OUTPUT);
  
  mySerial.begin(4800);

  Serial.begin(115200);

  Serial.flush();
}

void loop() {
  while(!(Serial.available()));

  digitalWrite(BUSY, LOW);

  while (!(mySerial.available()));
  Serial.print(" ");
  Serial.print((mySerial.read() * (4095/255)));

  if(data_frame == 35){
    Serial.print("\n");
    data_frame = 0;
  }
  data_frame++;

  digitalWrite(BUSY, HIGH);
}
