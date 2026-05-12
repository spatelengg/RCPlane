#include <SPI.h>
/*
TX: Config
CE 17
CSN 4
SCK 18
MOSI 23
MISO 19
*/
#define CE 17
#define CSN 4

void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println("ESP32 nRF24L01 SPI Test");

  SPI.begin(18, 19, 23, CSN); // SCK, MISO, MOSI, CSN

  pinMode(CSN, OUTPUT);
  pinMode(19, INPUT_PULLUP); 
  digitalWrite(CSN, HIGH);

  delay(100);

  uint8_t status = readRegister(0x07);

  Serial.print("STATUS register = 0x");
  Serial.println(status, HEX);

  if (status == 0x00) {
    Serial.println("❌ MISO stuck LOW");
  } 
  else if (status == 0xFF) {
    Serial.println("❌ MISO floating");
  } 
  else {
    Serial.println("✅ SPI WORKING!");
  }
}

uint8_t readRegister(uint8_t reg) {
  digitalWrite(CSN, LOW);
  delayMicroseconds(5);

  SPI.transfer(reg);
  uint8_t value = SPI.transfer(0xFF);

  digitalWrite(CSN, HIGH);
  return value;
}

void loop() {}