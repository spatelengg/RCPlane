#include <SPI.h>
#include <Wire.h>
#include <RF24.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <Adafruit_MCP23X17.h>

//MCP23017 pins
#define GPA0 0
#define GPA1 1
#define GPA2 2
#define GPA3 3
#define GPA4 4
#define GPA5 5
#define GPA6 6
#define GPA7 7
#define GPB0 8
#define GPB1 9
#define GPB2 10
#define GPB3 11
#define GPB4 12
#define GPB5 13
#define GPB6 14
#define GPB7 15


//Controls
#define JS1_X 36
#define JS1_Y 39
#define JS2_X 32
#define JS2_Y 33
#define POT_1 34
#define POT_2 35

#define SW_1 GPA1
#define SW_2 GPA2
#define SW_3 GPB1
#define SW_4 GPB2

#define MS1_U GPA3
#define MS1_D GPA4
#define MS1_R GPA5
#define MS1_L GPA6
#define MS1_P GPA7
#define MS2_U GPB3
#define MS2_D GPB4
#define MS2_R GPB5
#define MS2_L GPB6
#define MS2_P GPB7



#define NRF_CE 17
#define NRF_CSN 4
const byte address[6] = "RC001";
RF24 radio(NRF_CE, NRF_CSN);

Adafruit_MCP23X17  mcp;

// -------- TFT PINS --------
#define TFT_CS   21
#define TFT_DC   26    // A0
#define TFT_RST  25

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

uint16_t tx_id = 0x1234;
uint16_t packet_id = 0;

bool armed = false;

uint32_t arm_timer = 0;

#define TRIM_STEP 5 
int16_t trims[3] = {0,0,0};

float   rxBattery;
uint32_t lastUI = 0;
int rssi;

#define MAGIC 0xAB
struct TxPacket {
  uint8_t magic;
  uint16_t tx_id;
  uint16_t packet_id;
  uint16_t channels[12];
  //bool  switches[6];
  uint8_t flags;
  uint16_t crc;
};

struct RxTelemetry {
  uint16_t rx_batt_mv;
  int8_t rssi;
  uint8_t flags;
  uint16_t crc;
};

uint16_t crc16(const uint8_t *data, uint8_t len) {
  uint16_t crc = 0xFFFF;
  while (len--) {
    crc ^= *data++;
    for (uint8_t i=0;i<8;i++)
      crc = (crc & 1) ? (crc>>1) ^ 0xA001 : crc>>1;
  }
  return crc;
}

uint16_t readADC(uint8_t pin) {
  return map(analogRead(pin), 0, 4095, 1000, 2000);
}
float readTxBattery() {
  float adc = analogRead(27);
  float voltage = (adc / 4095.0) * 3.3 * 2.12;
  return voltage;
}
void setup() {
  Serial.begin(115200);
  Wire.begin(21,22);

  // Configure pins
  for (uint8_t i = 0; i < 16; i++) {
    mcp.pinMode(i, INPUT_PULLUP);
  }

  for (int i=0;i<16;i++) mcp.pinMode(i, INPUT);

  initDisplay();

  if (!radio.begin()) {
    Serial.println("[ERROR] NRF24 not responding");
    //while (1);
  }
  radio.setPALevel(RF24_PA_HIGH);
  radio.setDataRate(RF24_250KBPS);
  radio.setRetries(5, 15);
  radio.enableAckPayload();
  radio.openWritingPipe(address);
  radio.stopListening();
}
void initDisplay() {
  tft.initR(INITR_BLACKTAB);
  tft.setRotation(1);   // landscape
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextWrap(false);
}
void handleTrim() {
  // Multi Switch 1 (GPA3–GPA7)
 
  if (mcp.digitalRead(MS1_R) == LOW) trims[0] += TRIM_STEP; // right
  if (mcp.digitalRead(MS1_L) == LOW) trims[0] -= TRIM_STEP; // left

  // Multi Switch 2 (GPB3–GPB7)
  if (mcp.digitalRead(MS2_U) == LOW) trims[1] += TRIM_STEP; // right
  if (mcp.digitalRead(MS2_D) == LOW) trims[1] -= TRIM_STEP; // left
  if (mcp.digitalRead(MS2_R) == LOW) trims[2] += TRIM_STEP; // up
  if (mcp.digitalRead(MS2_L) == LOW) trims[2] -= TRIM_STEP; // down

  trims[0] = constrain(trims[0], -100, 100);
  trims[1] = constrain(trims[1], -100, 100);
  trims[2] = constrain(trims[1], -100, 100);  
}

void handleArm(uint16_t throttle) {
  if (!armed) {
    if (throttle > 1900) arm_timer = millis();
    if (throttle < 1050 && arm_timer && millis()-arm_timer > 2000)
      armed = true;
  }
}

void drawArmStatus(bool armed) {
  tft.fillRect(0, 30, 160, 16, ST77XX_BLACK);
  tft.setCursor(0, 30);
  tft.setTextSize(2);

  if (armed) {
    tft.setTextColor(ST77XX_GREEN);
    tft.print("ARMED");
  } else {
    tft.setTextColor(ST77XX_RED);
    tft.print("DISARM");
  }
} 
void drawTrims() {
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_WHITE);

  tft.fillRect(0, 50, 160, 30, ST77XX_BLACK);

  tft.setCursor(0, 50);
  tft.printf("J1 X:%+03d Y:%+03d\n", trims[0], trims[1]);

  tft.setCursor(0, 62);
  tft.printf("J2 X:%+03d Y:%+03d", trims[2], trims[3]);
}
void drawBattery() {
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_CYAN);
  tft.fillRect(0, 0, 160, 15, ST77XX_BLACK);

  tft.setCursor(0, 0);
  tft.printf("TX:%.2fV  RX:%.2fV", readTxBattery(), rxBattery );
}
void drawRSSI(int rssi) {
  tft.fillRect(0, 85, 160, 12, ST77XX_BLACK);
  tft.setCursor(0, 85);
  tft.setTextColor(ST77XX_YELLOW);
  tft.printf("RSSI: %d%%", rssi);
}
void updateDisplay() {
  if (millis() - lastUI < 200) return;
  lastUI = millis();

  drawBattery();
  drawArmStatus(armed);
  drawTrims();
  drawRSSI(rssi);
}

void loop() {
  handleTrim();

  TxPacket pkt{};
  pkt.magic = MAGIC;
  pkt.tx_id = tx_id;
  pkt.packet_id = packet_id++;

  pkt.channels[0] = readADC(JS1_X); // throttle
  pkt.channels[1] = readADC(JS1_Y) + trims[0];
  pkt.channels[2] = readADC(JS2_X) + trims[1];
  pkt.channels[3] = readADC(JS2_Y) + trims[2];
  pkt.channels[4] = readADC(POT_1);
  pkt.channels[5] = readADC(POT_2);

  pkt.channels[6]  = mcp.digitalRead(MS1_P) ? 2000 : 1000;
  pkt.channels[7]  = mcp.digitalRead(SW_1) ? 2000 : 1000;
  pkt.channels[8]  = mcp.digitalRead(SW_2) ? 2000 : 1000;
  pkt.channels[9]  = mcp.digitalRead(SW_3) ? 2000 : 1000;
  pkt.channels[10] = mcp.digitalRead(SW_4) ? 2000 : 1000;
  pkt.channels[11] = mcp.digitalRead(MS2_P) ? 2000 : 1000;
 

  handleArm(pkt.channels[0]);
  pkt.flags = armed ? 1 : 0;

  pkt.crc = crc16((uint8_t*)&pkt, sizeof(pkt)-2);

  radio.write(&pkt, sizeof(pkt));

  if (radio.isAckPayloadAvailable()) {
    RxTelemetry tel;
    radio.read(&tel, sizeof(tel));
    rxBattery = tel.rx_batt_mv / 1000;
    rssi = tel.rssi;
    // update display
  }
  updateDisplay();
  delay(10);
}
