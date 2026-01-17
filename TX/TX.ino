#include <SPI.h>
#include <RF24.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>

// -------- TFT PINS --------
#define TFT_CS   21
#define TFT_DC   26    // A0
#define TFT_RST  25

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

// ================= NRF24 =================
#define CE_PIN   17
#define CSN_PIN  5

RF24 radio(CE_PIN, CSN_PIN);
const byte address[6] = "RC001";

// ================= ADC PINS =================
#define J1_X_PIN 36
#define J1_Y_PIN 39
#define J1_S_PIN 27
#define J2_X_PIN 32
#define J2_Y_PIN 33
#define J2_S_PIN 14

// ================= DATA =================
struct RCData {
  uint16_t j1_x;
  uint16_t j1_y;
  bool     j1_s;
  uint16_t j2_x;
  uint16_t j2_y;
  bool     j2_s;  
};

RCData txData;

// ================= UTILS =================
uint16_t adcToPWM(int adc) {
  return map(adc, 0, 1900, 0, 255);
}


uint32_t lastDraw = 0;

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  delay(2000);

  

  Serial.println("\n==== ESP32 RC TRANSMITTER START ====");

  pinMode(J1_S_PIN, INPUT_PULLUP);
  pinMode(J2_S_PIN, INPUT_PULLUP);

  // ESP32 SPI (VSPI)
  // sck, miso, mosi, ss
  SPI.begin(18, 19, 23);
  tft.initR(INITR_BLACKTAB);   // Most ST7735R modules

  if (!radio.begin()) {
    Serial.println("[ERROR] NRF24 not responding");
    while (1);
  }
  radio.setAutoAck(true);
  radio.setPALevel(RF24_PA_MIN);
  radio.setDataRate(RF24_250KBPS);
  
  radio.setChannel(108);
  radio.setRetries(5, 15);
  radio.openWritingPipe(address);
  radio.stopListening();


  
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLACK);

  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(1);

  tft.setCursor(25, 5);
  tft.println("RC TRANSMITTER");

  // ADC setup
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);  // Allows full 0–3.3V range

  Serial.println("[OK] NRF24 ready");
  Serial.println("==================================");
}

// ================= LOOP =================
void loop() {

  // Read ADCs
  txData.j1_x = analogRead(J1_X_PIN);
  txData.j1_y = analogRead(J1_Y_PIN);
  txData.j2_x = analogRead(J2_X_PIN);
  txData.j2_y = analogRead(J2_Y_PIN);
  txData.j1_s = digitalRead(J1_S_PIN);
  txData.j2_s = digitalRead(J2_S_PIN);

  // Send packet
  bool ok = radio.write(&txData, sizeof(txData));

  // Debug (rate-limited)
  static unsigned long lastLog = 0;
  if (millis() - lastLog > 200) {
    lastLog = millis();

    Serial.print("[TX] ");
    Serial.print(ok ? "**OK " : "FAIL ");
    Serial.print("J1_X:");
    Serial.print(txData.j1_x);
    Serial.print(" J1_Y:");
    Serial.print(txData.j1_y);
    Serial.print(" J1_S:");
    Serial.print(txData.j1_s);
    Serial.print(" J2_X:");
    Serial.print(txData.j2_x);
    Serial.print(" J2_Y:");
    Serial.print(txData.j2_y);
    Serial.print(" J2_S:");
    Serial.println(txData.j2_s);
  }

  // ---- Controlled refresh ----
  if (millis() - lastDraw > 100) {
    lastDraw = millis();

    drawValue(10, 40, "J1X", txData.j1_x);
    drawValue(10, 55, "J1Y", txData.j1_y);
    drawValue(10, 70, "J2X", txData.j2_x);
    drawValue(10, 85, "J2Y", txData.j2_y);
    drawStatus();
  }

  delay(20);  // ~50Hz update
}

void drawValue(int x, int y, const char* label, uint16_t value) {
  // Clear line area (adjust width if needed)
  tft.fillRect(x, y, 140, 10, ST77XX_BLACK);

  tft.setCursor(x, y);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(1);

  tft.print(label);
  tft.print(": ");
  tft.print(value);
}

void drawStatus() {
  tft.fillRect(0, 130, 160, 30, ST77XX_BLACK);

  tft.setCursor(0, 130);
  //tft.printf("OK:%lu FL:%lu", okCnt, failCnt);

  tft.setCursor(110, 130);
  //tft.setTextColor(linkOK ? ST77XX_GREEN : ST77XX_RED);
  //tft.print(linkOK ? "LINK" : "FAIL");
  tft.setTextColor(ST77XX_WHITE);
}
