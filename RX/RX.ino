#include <SPI.h>
#include <RF24.h>
#include <Servo.h>

// ================= PINS =================
#define CSN_PIN  5
#define CE_PIN   6

#define M1_A 9    // GP9
#define M1_B 10   // GP10
#define M2_A 13   // GP13
#define M2_B 14   // GP14

#define SERVO_PIN 7 // GP7

#define VBAT_PIN 29        //GP29 - ADC pin
#define ADC_REF 3.3
#define ADC_MAX 4095.0    // 12-bit ADC
#define VBAT_DIVIDER 2.0  // 100k / 100k

// ================= OBJECTS =================
RF24 radio(CE_PIN, CSN_PIN);
Servo elevatorServo;

// ================= RF =================
const byte address[6] = "RC001";

// ================= DATA =================
/*
struct RCData {
  uint16_t throttle;
  uint16_t elevator;
  uint16_t rudder;
  uint16_t aux;
};*/
struct RCData {
  uint16_t j1_x;
  uint16_t j1_y;
  bool     j1_s;
  uint16_t j2_x;
  uint16_t j2_y;
  bool     j2_s;  
};

RCData rxData;

unsigned long lastPacketTime = 0;
unsigned long lastLogTime = 0;
bool failsafeActive = true;

 

void stopMotors() {
  analogWrite(M1_A, 0);
  analogWrite(M1_B, 0);
  analogWrite(M2_A, 0);
  analogWrite(M2_B, 0);
}


float readBatteryVoltage() {
  uint16_t raw = analogRead(VBAT_PIN);
  float v = (raw * ADC_REF / ADC_MAX) * VBAT_DIVIDER;
  return v;
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  delay(1500);

  Serial.println("\n==== RP2040 RC RECEIVER START ====");

  pinMode(VBAT_PIN, INPUT);
  analogReadResolution(12);

  pinMode(M1_A, OUTPUT);
  pinMode(M1_B, OUTPUT);
  pinMode(M2_A, OUTPUT);
  pinMode(M2_B, OUTPUT);

  stopMotors();
  Serial.println("[INIT] Motors stopped");

  elevatorServo.attach(SERVO_PIN);
  elevatorServo.writeMicroseconds(1500);
  Serial.println("[INIT] Servo centered (1500us)");

  // IMPORTANT: Use YOUR SPI pins
  // SCK=GP2, MISO=GP4, MOSI=GP3
  SPI.setSCK(2);   // GP2
  SPI.setTX(3);    // MOSI → GP3
  SPI.setRX(4);    // MISO → GP4
  SPI.begin();  

  if (!radio.begin()) {
    Serial.println("[ERROR] NRF24L01 not responding!");
  } else {
    Serial.println("[OK] NRF24L01 detected");
  }
  
  radio.setAutoAck(true);
  radio.setPALevel(RF24_PA_MIN);
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(108);
  radio.openReadingPipe(0, address);
  radio.startListening();

  Serial.println("[RF] Listening on address RC001.");
  Serial.println("=================================\n");
}

// ================= LOOP =================
void loop() {
  static unsigned long lastVBatLog = 0;

  if (millis() - lastVBatLog > 1000) {
    lastVBatLog = millis();

    float vbat = readBatteryVoltage();
/*
    Serial.print("[VBAT] ");
    Serial.print(vbat, 2);
    Serial.print(" V");

    if (vbat < 3.3) {
      Serial.println("  ❌ CRITICAL");
    } else if (vbat < 3.5) {
      Serial.println("  ⚠ WARNING");
    } else {
      Serial.println("  OK");
    }
*/
  }

  
  // -------- Packet Received --------
  if (radio.available()) {
    //Serial.print("Reading...");
    radio.read(&rxData, sizeof(rxData));
    //return;
    //Serial.println(" Read Done");
    lastPacketTime = millis();

    if (failsafeActive) {
      Serial.println("[RF] Signal restored");
      failsafeActive = false;
    }

     

    // ---- Motors ----
    //analogWrite(M1_A, throttlePWM);
    //analogWrite(M1_B, 0);
    //analogWrite(M2_A, throttlePWM);
    //analogWrite(M2_B, 0);

    // ---- Servo ----
     

    // ---- Rate-limited logging ----
    if (millis() - lastLogTime > 200) {
      lastLogTime = millis();

      Serial.print("[RX] J1_X:");
      Serial.print(rxData.j1_x);

      Serial.print(" | J1_Y:");
      Serial.print(rxData.j1_y);

      Serial.print(" | J2_X:");
      Serial.print(rxData.j2_x);

      Serial.print(" | J2_Y:");
      Serial.println(rxData.j2_y);
    }
  }
  else{
    //Serial.println("Data not available");
  }

  // -------- FAILSAFE --------
  if (millis() - lastPacketTime > 120) {
    if (!failsafeActive) {
      Serial.println("[FAILSAFE] Signal lost → Motors OFF, Servo CENTER");
      stopMotors();
      elevatorServo.writeMicroseconds(1500);
      failsafeActive = true;
    }
  }
}
