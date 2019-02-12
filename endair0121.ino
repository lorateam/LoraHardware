#include <SPI.h>
#include <LoRa.h>
#include "SSD1306.h" 
#include "images.h"
#include <Arduino.h>
#include "crc.h"

// Pin definetion of WIFI LoRa 32
// HelTec AutoMation 2017 support@heltec.cn 
#define SCK     5    // GPIO5  -- SX1278's SCK
#define MISO    19   // GPIO19 -- SX1278's MISO
#define MOSI    27   // GPIO27 -- SX1278's MOSI
#define SS      18   // GPIO18 -- SX1278's CS
#define RST     14   // GPIO14 -- SX1278's RESET
#define DI0     26   // GPIO26 -- SX1278's IRQ(Interrupt Request)
#define SDA    4
#define SCL   15
#define SD1306RST   16 //RST must be set by software

#define BAND    433E6  //you can set band here directly,e.g. 868E6,915E6
#define PABOOST true


SSD1306  display(0x3c, SDA, SCL, SD1306RST);

uint16_t ChipID;               // address of this device
byte destination = 0x00;      // destination of the gateway
#define PACKLEN 19
unsigned char g_bufPacket[PACKLEN];
char logbuf[256];
int setupInterval = 600000;
long lastSetupTime;

typedef struct _AIR_PARAMS 
{
  int nCO2;
  float fTVOC;
  float fCH2O;
  int nPM25;
  int nPM10;
  float fHUMIDITY;
  float fTEMPERATURE;
}AIR_PARAMS;

void printFrame(unsigned char* buf, int nLen)
{
  Serial.print("Data received: ");
  for (int n=0; n<nLen; n++)  {
    snprintf(logbuf, 256, "%02X ", buf[n]);  
    Serial.print(logbuf);
  }
  Serial.println("");
}

void parseFrame(unsigned char* buf, int nLen, AIR_PARAMS& ap)
{
  ap.nCO2 = buf[3]*256+buf[4];
  ap.fTVOC = (buf[5]*256.0+buf[6])/10;
  ap.fCH2O = (buf[7]*256.0+buf[8])/10;
  ap.nPM25 = buf[9]*256+buf[10];
  ap.nPM10 = buf[15]*256+buf[16];
  ap.fHUMIDITY = -6 + 125*(buf[11]*256.0+buf[12])/65536;
  ap.fTEMPERATURE = -46.85 + 175.72*(buf[13]*256.0+buf[14])/65536;
}

bool getCompleteFrame(unsigned char chRcv, unsigned char* buf)
{
  for(int i=0; i<PACKLEN-1; i++)
    buf[i] = buf[i+1];
  buf[PACKLEN-1] = chRcv;
  if(buf[0] == 0x01 && buf[1] == 0x03 && buf[2] == 0x0e)  {
    if (CRC_Check(buf, PACKLEN))  {
      return true;
    }
  }
  return false;
}

void logo(){
  display.clear();
  display.drawXbm(0,5,logo_width,logo_height,(uint8_t*)logo_bits);
  display.display();
}

void setup()
{
  pinMode(16,OUTPUT);
  digitalWrite(16, LOW);    // set GPIO16 low to reset OLED
  delay(50); 
  digitalWrite(16, HIGH); // while OLED is running, must set GPIO16 in high、
  display.init();
  display.flipScreenVertically();  
  display.setFont(ArialMT_Plain_10);
  logo();
  delay(1000);
  display.clear();
  Serial.begin(115200);                   // initialize serial
  Serial2.begin(9600, SERIAL_8N1, 17, 2); 
  while (!Serial);
  Serial.println("LoRa Duplex");
  
  SPI.begin(SCK,MISO,MOSI,SS);
  LoRa.setPins(SS,RST,DI0);
  
  if (!LoRa.begin(BAND,PABOOST)) {
    display.drawString(0, 0, "Starting LoRa failed!");
    display.display();
    while (1);
  }
  display.drawString(0, 0, "LoRa Initial success!");
  display.drawString(0, 10, "Wait for incoming data...");
  display.display();
  delay(1000);
  uint64_t chipid = ESP.getEfuseMac();
  ChipID = (chipid >> 32);
  snprintf(logbuf, 256, "ChipID=%08X", ChipID);  
  Serial.println(logbuf);
  //Serial.print(String(chip,HEX));
  //ChipID=(uint16_t)ESP.getEfuseMac();
  //Serial.print("to"+String((uint16_t)ChipID,HEX));
}

uint16_t charToInt(char a[8])
{
  uint16_t b = (a[0] - 48)*8*8*8*8*8*8*8 + (a[1] - 48)*8*8*8*8*8*8 + (a[2] - 48)*8*8*8*8*8 + (a[3]-48)*8*8*8*8 + (a[4]-48)*8*8*8 + (a[5]-48)*8*8 + (a[6]-48)*8 + a[7]-48;
  return b;
}

void Reply(int packetSize)
{
  if (packetSize == 0) return;          // if there's no packet, return

   // read packet header bytes:
  byte sender = LoRa.read();          // sender address
  char Recipient[8];
  for(int i = 0;i < 8;i ++)
  {
    Recipient[i] = LoRa.read();
  } 
  for(int i = 0;i < 4;i ++)
  {
    Serial.print(Recipient[i]);
  }
  bool ForMe = 1;
  uint16_t RecipientID = charToInt(Recipient);
  Serial.print(String(RecipientID,HEX));
  Serial.print(String(ChipID,HEX));
  if(RecipientID != ChipID)
    ForMe = 0;

  // if the recipient isn't this device or broadcast,
  if (!ForMe) {
    Serial.println("This message is not for me.");
    return;                             // skip rest of function
  }

  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);  //对齐方式
  display.setFont(ArialMT_Plain_10);          //设置字体
  display.drawString(0 , 15 , "Received Packet!");
  display.display();
  // if message is for this device, or broadcast, print details:
  Serial.println("Received from: 0x" + String(sender, HEX));
  Serial.println("Sent to: 0x" + String(RecipientID, HEX));
  Serial.println();
  long nowtime = millis();
  while(millis() - nowtime < 300)
  {
    sendMessage();
    Serial.println("Sent to: 0x" + String(RecipientID, HEX));
   }
  //Serial.println("Sending " + fireSignal);
  delay(200);
}

void loop()
{
  if(millis() - lastSetupTime > setupInterval)
  {
    if (!LoRa.begin(BAND,PABOOST)) {
    display.drawString(0, 0, "Starting LoRa failed!");
    display.display();
    while (1);
    }
    display.drawString(0, 0, "LoRa Initial success!");
    display.drawString(0, 10, "Wait for incoming data...");
    display.display();
    delay(1000);
  }
  AirqualityDetection();
  // parse for a packet, and reply with the result:
  Reply(LoRa.parsePacket());
}

void sendMessage()
{
  Serial.print(logbuf);
  char chipID[8];
  sprintf(chipID, "%08o", ChipID);
  LoRa.beginPacket();                   // start packet
  LoRa.print(chipID);                   // add sender address
  LoRa.write(destination);              // add destination address
  LoRa.write(0);                        // normal message
  byte len;
  while(logbuf[len] != '\0')
  {
    len++;
  }
  LoRa.write(len);              
  LoRa.print(logbuf);                      // add payload
  LoRa.endPacket();                     // finish packet and send it
}

void AirqualityDetection(){
  if (Serial2.available() == 0)return;
  if (getCompleteFrame(Serial2.read(), g_bufPacket))  {
      AIR_PARAMS ap;
      //printFrame(g_bufPacket, PACKLEN);
      parseFrame(g_bufPacket, PACKLEN, ap);
      char strTVOC[8], strCH2O[8], strHumid[8], strTemp[8];
      dtostrf(ap.fTVOC, 4, 2, strTVOC);
      dtostrf(ap.fCH2O, 4, 2, strCH2O);
      dtostrf(ap.fHUMIDITY, 4, 2, strHumid);
      dtostrf(ap.fTEMPERATURE, 4, 2, strTemp);
      snprintf(logbuf, 256, "\tChipID[%04X]  CO2:%dppm \nTVOC:%sug/m3 CH2O:%sug/m3 \nPM2.5:%dug/m3 PM10:%dug/m3 \nHumidity:%s%% Tempera\nture:%sC", ChipID, ap.nCO2, strTVOC, strCH2O, ap.nPM25, ap.nPM10, strHumid, strTemp);  
      Serial.print(logbuf);
      display.clear();
      display.setTextAlignment(TEXT_ALIGN_LEFT);  //对齐方式
      display.setFont(ArialMT_Plain_10);          //设置字体
      display.drawString(0 , 0 , logbuf);
      display.display();
      snprintf(logbuf, 256, "\"%04X\":{\"co2\":%d,\"tvoc\":%s,\"ch2o\":%s,\"pm25\":%d,\"pm10\":%d,\"humid\":%s,\"temp\":%s}", ChipID, ap.nCO2, strTVOC, strCH2O, ap.nPM25, ap.nPM10, strHumid, strTemp);  
      }
      //delay(5000);
}




