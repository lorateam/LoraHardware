#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>  
#include <WiFi.h>
#include "SSD1306.h" 
#include "images.h"
#include <PubSubClient.h>



// Pin definetion of WIFI LoRa 32
// HelTec AutoMation 2017 support@heltec.cn 
#define SCK     5    // GPIO5  -- SX1278's SCK
#define MISO    19   // GPIO19 -- SX1278's MISO
#define MOSI    27   // GPIO27 -- SX1278's MOSI
#define SS      18   // GPIO18 -- SX1278's CS
#define RST     14   // GPIO14 -- SX1278's RESET
#define DI00     26   // GPIO26 -- SX1278's IRQ(Interrupt Request) DI0 DI00???
#define SDA    4
#define SCL   15
#define SD1306RST   16 //RST must be set by software

#define BAND    433E6  //you can set band here directly,e.g. 868E6,915E6
#define PABOOST true

#define id_total  3  //the total number of ends

const char* ssid = "PHICOMM_BC_5G";
const char* password = "jishi520!";
const char* mqtt_server = "tx.3cat.top";

SSD1306  display(0x3c, SDA, SCL, SD1306RST);

byte localAddress = 0x00;     // address of this device
uint16_t SenderID = 0;
uint16_t ChipID[id_total]= {0x7009 , 0xA825 , 0xA02A};
int id_num=0;
byte poll = 0;
char buf[128];
int active;
uint64_t chipid; 

char mqtt_topic[128];
String PollingMsg = "{";
String ActiveMsg = "{";

byte turncount = 0;           // count of polling turn
long lastSendTime = 0;        // last send time
long lastPollingTime = 0;     // last polling time
int interval = 100;          // interval between sends
int pollingInterval = 60000;  // interval between polling

IPAddress g_ip;

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
char logbuf[256];
int value = 0;

void logo(){
  display.clear();
  display.drawXbm(0,5,logo_width,logo_height,(uint8_t*)logo_bits);
  display.display();
}

void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.print("Connecting to ");
  Serial.print(ssid);

  display.clear();
  //sprintf(buf, "SensorID:%08X", (uint16_t)(chipid >> 32));
  display.drawString(0, 0, buf);
  display.drawString(0, 12, "Connecting...");
  display.display();
  
  WiFi.begin(ssid, password);

  int nRetry = 20;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    if (nRetry -- <= 0) {
      //WiFi.disconnect();
      break;
    }
  }

  if (WiFi.status() != WL_CONNECTED)  {
    Serial.println("Connect failed!");
    return;
  }
  
  randomSeed(micros());

  Serial.print("\t");
  Serial.print("WiFi connected");
  Serial.print("IP address: ");
  g_ip = WiFi.localIP();
  Serial.println(WiFi.localIP());
  display.clear();
  //sprintf(buf, "SensorID:%08X", (uint16_t)(chipid >> 32));
  //display.drawString(0, 0, buf);
  display.drawString(0, 12, "WiFi connected");
  sprintf(buf, "%d.%d.%d.%d", g_ip[0],g_ip[1],g_ip[2],g_ip[3]); 
  Serial.print(buf);
  display.drawString(0, 24, buf);
  display.display();

}

void reconnect() {
    delay(2000);
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.print("connected\t");
      // Once connected, publish an announcement...
      //client.publish("testTopic", "Hello world, I am ESP8266!");
      // ... and resubscribe
      //client.subscribe("DCS/ESP8266/CTRL");//inTopic
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again ");
  }
}

void mqttReport() {
  if (!client.connected())  {
    Serial.print("MQTT Lost!  ");
    setup_wifi();
    while (!client.connected()) { delay(500); setup_wifi(); reconnect();}
    Serial.print("MQTT OK!");
    client.loop();
  }

  if(active)
  {
    snprintf(mqtt_topic, 128, "SCUMAKERS/FIRE/%d", active);  
    mqtt_topic[127] = '\0';
    Serial.print("MQTT Topic=");
    Serial.println(mqtt_topic);
    //Serial.print((g_bFired)?"\nPublish MQTT message: FIRE! \t":"\nPublish MQTT message: IDLE. \t");
    Serial.print("\nPublish MQTT message: FIRE! \t");
    ActiveMsg =ActiveMsg + '}';
    const char* msg = ActiveMsg.c_str();
    Serial.print(msg);
    if(client.publish(mqtt_topic, msg) == 0)
      Serial.print("\nPublish unsuccessfully!\t");
    else
      Serial.print("\nPublish successfully!\t");
    ActiveMsg = "{";
    display.clear();
    //sprintf(buf, "SensorID:%08X", (uint16_t)(chipid >> 32));
    //display.drawString(0, 0, buf);
    display.drawString(0, 0, "WiFi connected");
    sprintf(buf, "%d.%d.%d.%d", g_ip[0],g_ip[1],g_ip[2],g_ip[3]); 
    display.drawString(0, 20, buf);
    display.setFont(ArialMT_Plain_16);
    display.drawString(0, 40, String(SenderID,HEX) + "FIRE!");
    display.setFont(ArialMT_Plain_10);
    display.display();
  }
  else
  {
    snprintf(mqtt_topic, 128, "SCUMAKERS/FIRE/%d", active);  
    mqtt_topic[127] = '\0';
    Serial.print("MQTT Topic=");
    Serial.println(mqtt_topic);
    //Serial.print((g_bFired)?"\nPublish MQTT message: FIRE! \t":"\nPublish MQTT message: IDLE. \t");
    Serial.print("\nPublish MQTT message: NORMAL \t");
    PollingMsg = PollingMsg + "}";
    const char* msg = PollingMsg.c_str();
    Serial.print(msg);
    if(client.publish(mqtt_topic, msg) == 0)
      Serial.print("\nPublish unsuccessfully!\t");
    else
      Serial.print("\nPublish successfully!\t");
      display.clear();
    //sprintf(buf, "SensorID:%08X", (uint16_t)(chipid >> 32));
    //display.drawString(0, 0, buf);
    display.drawString(0, 0, "WiFi connected");
    sprintf(buf, "%d.%d.%d.%d", g_ip[0],g_ip[1],g_ip[2],g_ip[3]); 
    display.drawString(0, 20, buf);
    display.setFont(ArialMT_Plain_16);
    display.drawString(0, 40, (active)?"FIRE!":"IDLE");
    display.setFont(ArialMT_Plain_10);
    display.display();
  }
}

void setup() {
  Serial.begin(115200);                   // initialize serial
  while (!Serial);

  Serial.println("LoRa Duplex");
  
  pinMode(16,OUTPUT);
  pinMode(25,OUTPUT);
  
  digitalWrite(16, LOW);    // set GPIO16 low to reset OLED
  delay(50); 
  digitalWrite(16, HIGH); // while OLED is running, must set GPIO16 in high
  
  display.init();
  display.flipScreenVertically();  
  display.setFont(ArialMT_Plain_10);
  logo();
  delay(1500);
  display.clear();

  chipid=ESP.getEfuseMac()/4294967296;

  snprintf(mqtt_topic, 128, "SCUMAKERS/FIRE/%08X", (uint32_t)(chipid >> 16));  
  mqtt_topic[127] = '\0';
  Serial.print("MQTT Topic=");
  Serial.println(mqtt_topic);
  
  SPI.begin(SCK,MISO,MOSI,SS);
  LoRa.setPins(SS,RST,DI00);

  setup_wifi();
  
  client.setServer(mqtt_server, 1883);
  while (!client.connected()) {
      delay(500);
      setup_wifi();
      reconnect();
  }
  Serial.print("MQTT OK!  ");
  client.loop();
  
  if (!LoRa.begin(BAND,PABOOST)) {
    display.drawString(0, 0, "Starting LoRa failed!");
    display.display();
    while (1);
  }
  display.drawString(0, 0, "LoRa Initial success!");
  display.display();

}
void loop()
{
  if(millis() - lastPollingTime > pollingInterval)
  {
    for(int i = 0,i < 50,i++)
    {
      sendMessage(ChipID[id_num]);
      Serial.println("Sending to"+ String(ChipID[id_num],HEX));
      display.init();
      display.flipScreenVertically();  
      display.setFont(ArialMT_Plain_10);
      display.drawString(0, 0, "Sending to" + String(ChipID[id_num],HEX));
      display.display();
      //interval = random(20000) + 10000;    // 2-3 seconds
      // parse for a packet, and call onReceive with the result:
      if(onReceive(LoRa.parsePacket()) == 0)
        break;
    }
  }


  if(poll == 1)
  { 
    mqttReport();
    lastPollingTime = millis();
    PollingMsg = "{";
    poll = 0;
  }
}

void sendMessage(uint16_t ChipIdThisTime)
{
  char chipID[8];
  sprintf(chipID, "%08o", ChipIdThisTime);
  LoRa.beginPacket();                   // start packet
  LoRa.write(localAddress);             // add sender address
  //Serial.print(chipID.length());
  LoRa.print(chipID);// add destination address
  LoRa.endPacket();                     // finish packet and send it
}


int onReceive(int packetSize)
{
  if (packetSize == 0)                  // if there's no packet, return
  {
    return 1; 
  }
  Serial.print("received");
  display.clear();         
  display.init();
  display.flipScreenVertically();  
  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 15, "Received Packet");
  display.display();
  // read packet header bytes:
  char sender[8];
  for(int i = 0;i < 8;i ++)
  {
    sender[i] = LoRa.read();
  } 
  SenderID = charToInt(sender);
  int recipient = LoRa.read();          // recipient address
  active = LoRa.read();
  byte incomingLength = LoRa.read();
  String incoming = "";
   while (LoRa.available())
  {
    incoming += (char)LoRa.read();
  }
  Serial.print(incoming);
  if (incomingLength != incoming.length())
  {   // check length for error
    Serial.println("error: message length does not match length");
    Serial.print(incoming.length());
    Serial.print(incomingLength);
    return 1;                             // skip rest of function
  }

  // if the recipient isn't this device or broadcast,
  if (recipient != localAddress) {
    Serial.println("This message is not for me.");
    return 1;                             // skip rest of function
  }
  bool normal = true;
  if(SenderID != ChipID[id_num])
    normal = false;
  // if message is for this device, or broadcast, print details:
  //Serial.println("Result: " + active);
  //Serial.println("normal " + normal?"ture":"false");
  Serial.println();
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);
  display.drawString(0 , 0 , String(SenderID,HEX));
  if(active == 0)
    display.drawString(6 , 0 , "not active");
  else
    display.drawString(6 , 0 , "active");
  display.display();
  if(active == 1) //Report voluntarily
  {
    ActiveMsg =ActiveMsg + incoming;
    mqttReport();
    id_num = 0;
  }
  if(normal)
  {
    if(id_num < id_total - 1)
    {
      PollingMsg =PollingMsg + incoming + ','; 
      id_num ++;
      poll = 0;
    }
    else
    {
      PollingMsg =PollingMsg + incoming; 
      id_num = 0;
      poll = 1;
    }
  }
  return 0;
}

uint16_t charToInt(char a[8])
{
  uint16_t b = (a[0] - 48)*8*8*8*8*8*8*8 + (a[1] - 48)*8*8*8*8*8*8 + (a[2] - 48)*8*8*8*8*8 + (a[3]-48)*8*8*8*8 + (a[4]-48)*8*8*8 + (a[5]-48)*8*8 + (a[6]-48)*8 + a[7]-48;
  return b;
}

