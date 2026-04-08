//RX
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>
#include <Adafruit_NeoPixel.h>
#include <Ticker.h>

#define LED_PIN 16
#define LED_COUNT 1

// ================= PINS =================
#define CSN_PIN  5
#define CE_PIN   6

#define M1_PWM 9
#define M1_DIR 10
#define M2_PWM 13
#define M2_DIR 14

#define SERVO_PIN 7

#define VBAT_PIN 29
#define VBAT2_PIN 27
#define ADC_REF 3.3
#define ADC_MAX 4095.0
#define VBAT_DIVIDER 2.0

RF24 radio(CE_PIN, CSN_PIN);
Adafruit_NeoPixel pixel(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
Servo elevator;
Ticker ledTicker;

// ================= DATA =================
const byte address[6] = "RC001";

struct RCData {
  uint8_t magic;
  uint16_t tx_id;
  uint16_t packet_id;
  uint16_t channels[12];
  uint8_t flags;
  uint16_t crc;
};

struct RxTelemetry {
  uint16_t rx_batt_mv;
  int8_t rssi;
  uint8_t flags;
  uint16_t crc;
};

RCData data;
RxTelemetry telemetry;

// ================= STATE =================
bool armed = false;
bool failsafe = true;

uint32_t lastPacketTime = 0;

// smoothing
float motorL = 0;
float motorR = 0;
#define MOTOR_SMOOTH 0.3

// LED update flag (IMPORTANT)
volatile bool ledUpdateFlag = false;

// ================= FUNCTIONS =================
void statusGreen(){
  pixel.setPixelColor(0, pixel.Color(0, 5, 0));
  pixel.show();
}

void statusRed(){
  pixel.setPixelColor(0, pixel.Color(5, 0, 0));
  pixel.show();
}

void statusBlue(){
  pixel.setPixelColor(0, pixel.Color(0, 0, 5));
  pixel.show();
}

void updateLED() {
  if (failsafe) {
    statusRed();
  }
  else if (armed) {
    statusGreen();
  }
  else {
    statusBlue();
  }
}

// ISR-safe ticker function
void ledTick() {
  ledUpdateFlag = true;
}

void stopMotors() {
  analogWrite(M1_PWM, 0);  
  analogWrite(M2_PWM, 0); 
}

void setMotor(int pinA, int speed) {
  analogWrite(pinA, speed);  
}

float readBatteryVoltage() {
  uint16_t raw = analogRead(VBAT2_PIN);
  return (raw * ADC_REF / ADC_MAX) * VBAT_DIVIDER;
}

bool linkLost = false;
uint32_t linkLostStart = 0;

// ================= SETUP =================
void setup() {
  Serial.begin(115200);

  // ✅ INIT LED FIRST
  pixel.begin();
  pixel.clear();
  pixel.show();

  statusRed();

  delay(5000);

  Serial.println("\n==== RP2040 RC RECEIVER START ====");

  analogWriteFreq(20000);

  pinMode(VBAT_PIN, INPUT);
  pinMode(VBAT2_PIN, INPUT);
  //analogReadResolution(12);

  pinMode(M1_PWM, OUTPUT);
  pinMode(M1_DIR, OUTPUT);
  pinMode(M2_PWM, OUTPUT);
  pinMode(M2_DIR, OUTPUT);

  analogWrite(M1_PWM, 0);  
  analogWrite(M2_PWM, 0); 
  digitalWrite(M1_DIR, LOW);
  digitalWrite(M2_DIR, LOW);

  stopMotors();

  elevator.attach(SERVO_PIN);
  //elevator.writeMicroseconds(1500);

  // SPI setup
  SPI.setSCK(2);
  SPI.setTX(3);
  SPI.setRX(4);
  SPI.begin();

  if (!radio.begin()) {
    Serial.println("[ERROR] NRF24L01 not responding!");
  } else {
    Serial.println("[OK] NRF24L01 detected");
    statusGreen();
  }

  radio.begin();

  radio.setChannel(76);
  radio.setPALevel(RF24_PA_HIGH);
  radio.setDataRate(RF24_250KBPS);
  radio.setCRCLength(RF24_CRC_16);

  radio.setAutoAck(true);
  radio.setRetries(5,15);
  radio.enableAckPayload();

  radio.openReadingPipe(0, address);

  radio.startListening();

  Serial.println("[RF] Listening...");
  radio.printDetails();
  Serial.println("=================================\n");

  // ✅ START TICKER (SAFE)
  ledTicker.attach_ms(100, ledTick);
}

// ================= LOOP =================
void loop() {
  
  // ================= RECEIVE =================
  if (radio.available()) {
    radio.read(&data, sizeof(data));

    lastPacketTime = millis();
    failsafe = false;
    //Serial.print("Flag: ");
    //Serial.println(data.flags);

    bool txArmed = (data.flags == 1);
    int throttleRaw = data.channels[0];

    // CHANNELS
    int throttle = map(throttleRaw, 1000, 2000, 0, 255);
    int yaw      = map(data.channels[1], 1000, 2000, 100, -100);
    int pitch    = map(data.channels[3], 1000, 2000, 0, 180);
    if (abs(yaw) < 10) yaw = 0;

    // SAFE ARM
    if (txArmed && throttleRaw < 1050 && yaw == 0) {
      armed = true;
      //Serial.println("** ARMED **");
    }

    if (!txArmed) {
      armed = false;
      //Serial.println("** DIS-ARMED **");
    }

    int targetL = constrain(throttle + yaw, 0, 255);
    int targetR = constrain(throttle - yaw, 0, 255);

     

    // OUTPUT
    if (armed) {
      // SMOOTHING
      motorL += (targetL - motorL) * MOTOR_SMOOTH;
      motorR += (targetR - motorR) * MOTOR_SMOOTH;

      Serial.print("T: ");
      Serial.print(throttle);
      Serial.print(" | Y: ");
      Serial.print(yaw);
      Serial.print(" | M1: ");
      Serial.print(motorL);
      Serial.print(" | M2: ");
      Serial.println(motorR);

      setMotor(M1_PWM, motorL);
      setMotor(M2_PWM, motorR);
      //elevator.write(pitch);
    } else {
      stopMotors();
      //elevator.write(90);
    }
    elevator.write(pitch);

    // TELEMETRY
    telemetry.rx_batt_mv = readBatteryVoltage() * 1000;
    Serial.println(telemetry.rx_batt_mv);
    telemetry.rssi = 100;
    radio.writeAckPayload(0, &telemetry, sizeof(telemetry));
  }
  else {
    Serial.println("Radio not available");
  }

  // ================= FAILSAFE =================
  if (millis() - lastPacketTime > 1000) {
    if (!failsafe) {
      Serial.println("** FAILSAFE ACTIVATED **");
    }

    failsafe = true;
    armed = false;

    stopMotors();
    elevator.write(90);
  }

  // ================= SAFE LED UPDATE =================
  if (ledUpdateFlag) {
    ledUpdateFlag = false;
    updateLED();
  }
  delay(50);
}
