//TX
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <Ticker.h>

#define ST77XX_DARKGREY 0x7BEF

// ================= CONTROLS =================
#define JS1_X 39
#define JS1_Y 36
#define JS2_X 33
#define JS2_Y 32

#define NRF_CE 17
#define NRF_CSN 4

#define TFT_CS   21
#define TFT_DC   26
#define TFT_RST  25

#define ARM_HOLD_TIME 1000

RF24 radio(NRF_CE, NRF_CSN);
Adafruit_ST7735 tft(TFT_CS, TFT_DC, TFT_RST);

Ticker timer_Transmit, timer_DisplayUpdate, timer_ThrottleUpdate;

// ================= FLAGS =================
volatile bool txFlag=false;
volatile bool displayFlag=false;
volatile bool throttleFlag=false;

// ================= STATE =================
uint16_t tx_id = 0x1234;
uint16_t packet_id = 0;
bool armed = false;

float throttle = 1000;
float rxBattery = 0;

bool armCommandActive = false;
uint32_t armStartTime = 0;

bool linkLost = false;
uint32_t linkLostStart = 0;

const byte address[6] = "RC001";

// ================= UI CACHE =================
int prevRSSI = -1;
float prevTxBat = -1, prevRxBat = -1;
bool prevArmed = false;

int prevJoy1X=-1, prevJoy1Y=-1;
int prevJoy2X=-1, prevJoy2Y=-1;
int prevThrottle=-1;

// ================= PACKETS =================
#define MAGIC 0xAB

struct TxPacket {
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

TxPacket pkt;
RxTelemetry telemetry;

// ================= DISPLAY =================
void drawStaticUI() {

  tft.drawRect(0, 0, 160, 14, ST77XX_WHITE);
  tft.drawRect(0, 16, 70, 20, ST77XX_WHITE);
  tft.drawRect(75, 16, 60, 40, ST77XX_WHITE);

  tft.drawRect(5, 60, 60, 60, ST77XX_WHITE);
  tft.drawRect(75, 60, 60, 60, ST77XX_WHITE);

  tft.drawRect(140, 16, 18, 100, ST77XX_WHITE);
}

void initDisplay() {

  tft.initR(INITR_BLACKTAB);
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextWrap(false);

  drawStaticUI();
}

// ================= UI ELEMENTS =================

void drawBattery(float tx, float rx) {

  if (abs(tx-prevTxBat)<0.05 && abs(rx-prevRxBat)<0.05) return;

  prevTxBat=tx;
  prevRxBat=rx;

  tft.setTextSize(1);
  tft.fillRect(2,2,120,10,ST77XX_BLACK);

  tft.setCursor(2,2);
  tft.setTextColor(ST77XX_CYAN);
  tft.printf("TX:%.2f RX:%.2f",tx,rx);
}

void drawArmStatus(bool state) {

  if(state==prevArmed) return;

  prevArmed=state;

  tft.fillRect(2,18,66,16,ST77XX_BLACK);

  tft.setCursor(5,20);
  tft.setTextSize(2);
  tft.setTextColor(state?ST77XX_GREEN:ST77XX_RED);
  tft.print(state?"ARM":"SAFE");
}

void drawJoystick(int x,int y,int rawX,int rawY,int &px,int &py){

  int jx=map(rawX,1000,2000,0,58);
  int jy=map(rawY,1000,2000,0,58);

  if(jx==px && jy==py) return;

  if(px!=-1)
    tft.fillCircle(x+px,y+py,3,ST77XX_BLACK);

  tft.drawFastHLine(x,y+29,60,ST77XX_DARKGREY);
  tft.drawFastVLine(x+29,y,60,ST77XX_DARKGREY);

  tft.fillCircle(x+jx,y+jy,3,ST77XX_WHITE);

  px=jx;
  py=jy;
}

// ================= HORIZON =================

void drawHorizon(int pitch, int roll){

  int cx=105;
  int cy=36;

  tft.fillRect(76,17,58,38,ST77XX_BLACK);

  int yOffset=map(pitch,1000,2000,-15,15);

  tft.drawLine(cx-25,cy+yOffset,cx+25,cy+yOffset,ST77XX_CYAN);
  tft.drawCircle(cx,cy,3,ST77XX_WHITE);
}

// ================= THROTTLE =================

void drawThrottle(float val){

  int y=map(val,1000,2000,100,0);
  if(y==prevThrottle) return;

  prevThrottle=y;

  int x=142;
  int h=96;

  tft.fillRect(x+1,17,16,h,ST77XX_BLACK);

  int fillHeight=map(val,1000,2000,0,h);

  uint16_t color=armed?ST77XX_GREEN:ST77XX_RED;

  tft.fillRect(x+1,17+(h-fillHeight),16,fillHeight,color);

  int knobY=17+(h-fillHeight);

  tft.fillRect(x-2,knobY-2,22,4,ST77XX_WHITE);
}

// ================= RSSI =================

void drawRSSI(int rssi){

  if(rssi==prevRSSI) return;

  prevRSSI=rssi;

  tft.fillRect(120,2,38,10,ST77XX_BLACK);

  int bars=map(rssi,0,100,0,5);

  for(int i=0;i<bars;i++)
    tft.fillRect(120+i*7,10-(i*2),5,i*2+2,ST77XX_GREEN);
}

// ================= DISPLAY UPDATE =================

void updateDisplay(){

  drawBattery(readTxBattery(),rxBattery);
  drawArmStatus(armed);

  drawJoystick(5,60,pkt.channels[1],pkt.channels[0],prevJoy1X,prevJoy1Y);
  drawJoystick(75,60,pkt.channels[2],pkt.channels[3],prevJoy2X,prevJoy2Y);

  drawHorizon(pkt.channels[3],pkt.channels[2]);
  drawThrottle(throttle);

  drawRSSI(telemetry.rssi);
}

// ================= LOGIC =================

void updateArming(){

  int yaw=(3000-map(readADC(JS1_X),0,4095,1000,2000));
  bool throttleLow=(throttle<=1050);

  if(throttleLow && yaw>1900){

    if(!armCommandActive){
      armStartTime=millis();
      armCommandActive=true;
      Serial.println("ARM HOLD START");
    }

    if(millis()-armStartTime>ARM_HOLD_TIME){
      armed=true;
      Serial.println("SYSTEM ARMED");
    }

  }else{
    armCommandActive=false;
  }

  if(!throttleLow)
    armCommandActive=false;
}

float readTxBattery(){

  float adc=analogRead(27);
  return (adc/4095.0)*3.3*3*0.83;
}

uint16_t crc16(const uint8_t *data,uint16_t len){

  uint16_t crc=0xFFFF;

  while(len--){
    crc^=*data++;

    for(uint8_t i=0;i<8;i++)
      crc=(crc&1)?(crc>>1)^0xA001:crc>>1;
  }

  return crc;
}

uint16_t readADC(uint8_t pin){

  static uint16_t prev[40];

  uint16_t val=analogRead(pin);

  prev[pin]=(prev[pin]*3+val)/4;

  return prev[pin];
}

// ================= TASKS =================

void task_ThrottleUpdate(){

  int joy=readADC(JS1_Y);
  float diff=joy-1910;

  if(abs(diff)>80){

    float n=diff/2048.0;
    float curve=n*abs(n);

    throttle+=curve*50*0.5;
  }

  if(throttle>2000) throttle=2000;
  if(throttle<1000) throttle=1000;
}

void task_Transmit(){

  updateArming();

  pkt.magic = MAGIC;
  pkt.tx_id = tx_id;
  pkt.packet_id = packet_id++;

  pkt.channels[0] = armed ? throttle : 1000;
  pkt.channels[1] = (3000 - map(readADC(JS1_X),0,4095,1000,2000));
  pkt.channels[2] = map(readADC(JS2_X),0,4095,1000,2000);
  pkt.channels[3] = map(readADC(JS2_Y),0,4095,1000,2000);

  pkt.flags = armed ? 1 : 0;
  pkt.crc = crc16((uint8_t*)&pkt,sizeof(pkt)-2);

  Serial.print("TX Packet: ");
  Serial.print(pkt.packet_id);
  Serial.print(" ARM:");
  Serial.print(armed);
  Serial.print(" THR:");
  Serial.println(pkt.channels[0]);

  bool ok = radio.write(&pkt,sizeof(pkt));

  if(ok){

    Serial.println("RF OK");

    // reset link loss timer
    linkLost = false;
    linkLostStart = 0;

    if(radio.isAckPayloadAvailable()){
      radio.read(&telemetry,sizeof(telemetry));
      rxBattery = telemetry.rx_batt_mv / 1000.0;

      Serial.print("RX BATTERY: ");
      Serial.println(rxBattery);
    }

  }
  else{

    Serial.println("RF FAIL");

    // start timer only once
    if(!linkLost){
      linkLost = true;
      linkLostStart = millis();
      Serial.println("LINK LOST TIMER START");
    }

    // trigger failsafe only after 1 second
    if(linkLost && (millis() - linkLostStart > 1000)){

      if(armed){
        Serial.println("SAFE MODE ACTIVATED");
      }

      throttle = 1000;
      armed = false;
    }
  }
}

void task_DisplayUpdate(){
  updateDisplay();
}

// ================= TICKERS =================

void tickTX(){txFlag=true;}
void tickDisplay(){displayFlag=true;}
void tickThrottle(){throttleFlag=true;}

// ================= SETUP =================

void setup(){

  Serial.begin(115200);
  Serial.println("RC TRANSMITTER START");

  initDisplay();
  
  SPI.begin(18,19,23); 

  radio.begin();
  radio.setChannel(76);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.enableAckPayload();
  //radio.enableDynamicPayloads();
  radio.setRetries(5,15);
  radio.openWritingPipe(address);
  radio.stopListening();

  radio.printDetails();

  timer_Transmit.attach_ms(100,tickTX);
  timer_DisplayUpdate.attach_ms(60,tickDisplay);
  timer_ThrottleUpdate.attach_ms(20,tickThrottle);
}

// ================= LOOP =================

void loop(){

  if(throttleFlag){
    throttleFlag=false;
    task_ThrottleUpdate();
  }

  if(txFlag){
    txFlag=false;
    task_Transmit();
  }

  if(displayFlag){
    displayFlag=false;
    task_DisplayUpdate();
  }
}
