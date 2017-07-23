#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <SPI.h>
#include <stdint.h>
#include <WebSocketsClient.h>
#include <Hash.h>
#include <ArduinoJson.h>
#include "FS.h"
#include "MFRC522.h"
//needed for library for smartconfig
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>         //https://github.com/tzapu/WiFiManager

extern "C" {
#include "user_interface.h"
}

/* wiring the MFRC522 to ESP8266 (ESP-12)
RST     = GPIO5
SDA(SS) = GPIO4
MOSI    = GPIO13
MISO    = GPIO12
SCK     = GPIO14
GND     = GND
3.3V    = 3.3V
*/

#define BOARD_TYPE              1

#define RST_PIN                 5  // RST-PIN für RC522 - RFID - SPI - Modul GPIO5
#define SS_PIN                  16  // SDA-PIN für RC522 - RFID - SPI - Modul GPIO4

#define DATA_PIN                4
#define CLK_PIN                 15
#define LATCH_PIN               2

#define BTN_PIN                 0

#define LED_UPDATE_INTERVAL     5 // 5 ms
#define CARD_SCAN_INTERLVAL     200 // 0,5 s
#define ONE_MINUTE              60000 //60000 // 60k ms
#define WASH_TIME               40 // 40 min
#define SEND_BROADCAST_CMD      1000
#define SEND_PING_CMD           2000

#define UDP_PORT                6969
#define TCP_PORT                9696

#define WIFI_LED                0
#define SEVER_LED               1
#define RUNNING_LED             2

#define DEBUG                   1

#ifdef DEBUG
 #define TRACE 
#else
 #define TRACE for(;0;)
#endif /* DEBUG */

os_timer_t myTimer;

//String tagID= "A1 32 71 8B";
String tagID= "20 12 F1 19";

const char *ssid;     // change according to your Network - cannot be longer than 32 characters!
const char *pass; // change according to your Network

char packetBuffer[255];          // buffer to hold incoming packet

const uint8_t number[10] = {0x3F,0x06,0x5B,0x4F,0x66,0x6D,0x7D,0x07,0x7F,0x6F}; // katot chung
// const uint8_t number[10] = {0xC0,0xF9,0xA4,0xB0,0x99,0x92,0x82,0xF8,0x80,0x90}; // anot chung

uint8_t g_led1 = 0;
uint8_t g_led2 = 0;
uint8_t g_led3 = 0;

bool isStarted = false;
bool isWifiConnected = false;
bool isServerConnected = false;
bool isGotIpServer = false;
bool isNewCardFound = false;
bool isSameCard = false;
bool state_role = false;
bool isRegisted = false;
bool requestWashMachineStop = false;
bool isRequested = true;
bool hasCardFront = false;
bool isEnteredConfig = false;

volatile byte state_button = false;

byte card_uid[4] = {0xA1, 0x32, 0x71, 0x8B};

uint8_t g_cardNotFoundCount = 0;
uint8_t g_cardFoundCount  = 0;

uint32_t g_prevLedUpdateTime = 0;
uint32_t g_prevCardScanTime = 0;
uint32_t g_prevHasCard = 0;
uint32_t g_prevMinuteTime = 0;
uint32_t g_prevSendBroadcastCmdTime = 0;
uint32_t g_prevSendPingTime = 0;
uint32_t g_operateTime = 0;
uint32_t g_pingSendCount = 0;
uint32_t g_washTime = 0;
//uint32_t deviceID = 0;
uint32_t g_cardID = 0;
uint32_t g_RunningCardID = 0;


uint8_t ledData = 0;
uint8_t ledCtrlData = 0;
uint8_t ledPos = 0;

MFRC522 mfrc522(SS_PIN, RST_PIN); // Create MFRC522 instance

WiFiUDP udpClient;
WebSocketsClient webSocket;
IPAddress broadcastIp;
IPAddress remoteIp;
IPAddress localIP;

String socketCmd = "";
String ESPID = String(ESP.getChipId());
void setup() {  
    Serial.begin(115200);    // Initialize serial communications
    //ESP.wdtDisable();
    /* WiFiManager */
    /* Local intialization. Once its business is done, there is no need to keep it around */
    //ESP.wdtDisable();
    //
    WiFiManager wifiManager;

    /* 
    fetches ssid and pass from eeprom and tries to connect if it does not connect it starts an access point with the specified name here "AutoConnectAP" 
    and goes into a blocking loop awaiting configuration wifiManager.autoConnect("AutoConnectAP", "password"); or use this for auto generated name ESP + ChipID 
    */
    
    /* 
    - when your ESP starts up, it sets it up in Station mode and tries to connect to a previously saved Access Point
    - if this is unsuccessful (or no previous network saved) it moves the ESP into Access Point mode and spins up a DNS and WebServer (default ip 192.168.4.1)
    - using any wifi enabled device with a browser (computer, phone, tablet) connect to the newly created Access Point
    - because of the Captive Portal and the DNS server you will either get a 'Join to network' type of popup or get any domain you try to access redirected to the configuration portal
    choose one of the access points scanned, enter password, click save
    - ESP will try to connect. If successful, it relinquishes control back to your app. If not, reconnect to AP and reconfigure.
    */
    wifiManager.autoConnect();
    
    
    //ESP.wdtEnable(150000);
    delay(100);

    
    TRACE Serial.println("\r\nInitialize....");
    delay(5000);
    
    // Init pin
    pinMode(DATA_PIN, OUTPUT);
    pinMode(CLK_PIN, OUTPUT);
    pinMode(LATCH_PIN, OUTPUT);
    pinMode(BTN_PIN, INPUT_PULLUP);
    
    SPI.begin();           // Init SPI bus
    mfrc522.PCD_Init();    // Init MFRC522
    mfrc522.PCD_SetAntennaGain(mfrc522.RxGain_max);

    //attachInterrupt(digitalPinToInterrupt(BTN_PIN), btnHandler, FALLING );

    digitalWrite(DATA_PIN, LOW);
    digitalWrite(CLK_PIN, LOW);
    digitalWrite(LATCH_PIN, LOW);
    
    delay(100);
    //timer0_detachInterrupt();
    udpClient.flush();
    udpClient.stopAll();
                        
    udpClient.begin(UDP_PORT);

    TRACE Serial.println(F("UDP Ready!"));
    
    setLedEnable(RUNNING_LED, false);
    setLedEnable(WIFI_LED, false);
    setLedEnable(SEVER_LED, false);
    setRelayEnable(false);
    
    delay(5000);
    //timer0_isr_init();
    //timer0_attachInterrupt(ledUpdate);
    //timer0_write(ESP.getCycleCount()+6000000);
    
    //timer1_disable();
    //timer1_isr_init();
    //timer1_attachInterrupt(ledUpdate);
    //timer1_enable(TIM_DIV1, TIM_EDGE, 1);
    //timer1_enable(TIM_DIV1, TIM_EDGE, TIM_LOOP);
    //timer1_write(ESP.getCycleCount()+600000);

    TRACE Serial.println(F("LED Ready!"));
}

void btnHandler(void)
{
    static byte i = false;
    if(state_role == true) state_role = false;
    else state_role = true;

    setRelayEnable(bool(state_role));
    state_button=true;
}

void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
    switch(type) {
        case WStype_DISCONNECTED:
            {
                //timer0_detachInterrupt();
                TRACE Serial.printf("[WSc] Disconnected!\n");
                isServerConnected = false;
                setLedEnable(SEVER_LED, false);
                isGotIpServer = false;
                isRegisted = false;
                webSocket.disconnect();
                udpClient.begin(UDP_PORT);
                //timer0_attachInterrupt(ledUpdate);                
            }
            break;
        case WStype_CONNECTED:
            {
                //timer0_detachInterrupt();
                TRACE Serial.printf("[WSc] Connected to url: %s\n",  payload);
                setLedEnable(SEVER_LED, true);
                isServerConnected = true;
                udpClient.flush();
                udpClient.stopAll();
                webSocket.sendTXT("5");
                //timer0_attachInterrupt(ledUpdate);
            }
            break;
        case WStype_TEXT:
            {
                if (length > 2)
                {
                    //timer0_detachInterrupt();
                    // 42["response","{\"code\":0}"]
                    if (payload[0] == '4' && payload[1] == '2')
                    {
                        String resData((const char*)payload);
                        TRACE Serial.println(resData);
                        
                        String temp((char*)payload);
                        /* Get Json field in string  */
                        /* Message request_type : "42["response",{"request_type":"wm_ask_for_run","code":2,"duration":35}]" */
                        
                        temp = temp.substring(temp.indexOf('{'), temp.indexOf('}')+1);
                        TRACE Serial.println(temp);
                        StaticJsonBuffer<200> jsonBuffer;
                        JsonObject& ipmessage = jsonBuffer.parseObject(temp);
                        if (!ipmessage.success()) {
                        TRACE Serial.println("parseObject() failed");
                        }
                        
                        else
                        {
                        const char* request_code = ipmessage["code"];
                        String tem_request_code((char*)request_code);
                        if(tem_request_code=="2" && isRequested)
                            {
                                const char* duration = ipmessage["duration"];
                                String tem_duration((char*)duration);
                                //TRACE Serial.print("duration : ");
                                //TRACE Serial.println(tem_duration);
                                g_washTime = tem_duration.toInt();
                                TRACE Serial.println("runing");
                                setLedEnable(RUNNING_LED, true);
                                digitUpdate(g_washTime);
                                isRequested = false;
                                isStarted = true ;                                  
                                setRelayEnable(true);
                            }
                        }
                    }
                    //timer0_attachInterrupt(ledUpdate);
                }
            }
            break;
        case WStype_BIN:
            break;
    }
}

void loop() {

    uint32_t curTime = millis();
    uint32_t packetSize = 0;
    uint32_t tmpCardId = 0;
    ledUpdate();
    if (WiFi.status() == WL_CONNECTED && isWifiConnected == false)
    {
        broadcastIp = ~WiFi.subnetMask() | WiFi.gatewayIP();
        isWifiConnected = true;
        setLedEnable(WIFI_LED, true);
        TRACE Serial.println("Connected");
    }
    else if (WiFi.status() != WL_CONNECTED && isWifiConnected == true)
    {
        TRACE Serial.println("Disconnected");
        setLedEnable(WIFI_LED, false);
        if (isServerConnected)
        {
            // disconnect
            isServerConnected = false;
        }
        isWifiConnected = false;
        isGotIpServer = false;
        isRegisted = false;
    }

    if (isWifiConnected && isGotIpServer)
    {
        //timer0_detachInterrupt();
        webSocket.loop();
        //timer0_attachInterrupt(ledUpdate);
    }

    // 100ms loop
    if (curTime - g_prevCardScanTime >= CARD_SCAN_INTERLVAL) // every 0.1s
    {
        g_prevCardScanTime = curTime;

        if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial())
        {
            tmpCardId = 0;
            g_cardNotFoundCount = 0;
            for (byte i = 0; i < mfrc522.uid.size; i++)
            {
                tmpCardId |= (mfrc522.uid.uidByte[i] << (i * 8));
                //TRACE Serial.print(mfrc522.uid.uidByte[i] < 0x10 ? " 0" : " ");
                //TRACE Serial.print(mfrc522.uid.uidByte[i], HEX);
            }
            if (tmpCardId != g_cardID) // new card found
            {
                if (isStarted)
                {
                    isStarted = false;
                    requestWashMachineStop = true;
                    digitUpdate(0);
                }
                g_cardID = tmpCardId;
                isNewCardFound = true;
            }
            //TRACE Serial.println();
        }
        else
        {
            if (g_cardID != 0)
            {
                if (g_cardNotFoundCount++ >= (20)) // 5s
                {
                    if (isStarted)
                    {
                        isStarted = false;
                        isRequested = true;
                        requestWashMachineStop = true;
                        digitUpdate(0);
                    }
                    g_cardNotFoundCount = 0;
                    g_cardID = 0;
                }
            }
        }
    }
    else
    {

    }

    UDPbroadcast :
    if (isWifiConnected)
    {
        if (!isGotIpServer)
        {
            if (curTime - g_prevSendBroadcastCmdTime >= SEND_BROADCAST_CMD)
            {
                //timer0_detachInterrupt();
                g_prevSendBroadcastCmdTime = curTime;
                TRACE Serial.println("broadcast msg");
                String broadcastMsg;
                localIP = WiFi.localIP();
                /* send broadcast cmd */
                broadcastMsg += "{\"client\":\"";
                broadcastMsg += localIP[0];
                broadcastMsg += '.';
                broadcastMsg += localIP[1];
                broadcastMsg += '.';
                broadcastMsg += localIP[2];
                broadcastMsg += '.';
                broadcastMsg += localIP[3];
                broadcastMsg += "\"}";
                udpClient.beginPacket(broadcastIp, UDP_PORT);
                udpClient.write(broadcastMsg.c_str());
                udpClient.endPacket();
                //timer0_attachInterrupt(ledUpdate);
            }
            else
            {
                packetSize = udpClient.parsePacket();
                if (packetSize)
                {
                    //timer0_detachInterrupt();
                    /* read the packet into packetBufffer */
                    int len = udpClient.read(packetBuffer, 255);
                    if (len > 0) {
                      packetBuffer[len] = 0;
                    }              
                    TRACE Serial.printf("%s\n", packetBuffer);
                    
                    String temp((char*)packetBuffer);
                    /* Get Json field in string  */
                    /* Message ip_server : "42[\"response\",{\"ip_server\":\"192.168.1.1\"}]" */
                    /* Message reset_setting : "42[\"response\",{\"reset_setting\":\"true\"}]" */
                    temp = temp.substring(temp.indexOf('{'), temp.indexOf('}')+1);
                    StaticJsonBuffer<200> jsonBuffer;
                    JsonObject& ipmessage = jsonBuffer.parseObject(temp);
                    if (!ipmessage.success()) {
                        TRACE Serial.println("parseObject() failed");
                        goto UDPbroadcast;
                    }
                    else
                    {
                    const char* reset_setting = ipmessage["reset_setting"];
                    String tem_reset_setting((char*)reset_setting);
                    if(tem_reset_setting=="true")
                    {
                        /* WiFiManager */
                        /* Local intialization. Once its business is done, there is no need to keep it around */
                        //timer0_detachInterrupt();
                        udpClient.flush();
                        udpClient.stopAll();
                        webSocket.disconnect();
                        TRACE Serial.println("reset setting SSID and Password");  
                        WiFiManager wifiManager;
                        /* reset saved settings */
                        
                        wifiManager.resetSettings();
                        WiFi.disconnect();
                        wifiManager.autoConnect();
                        udpClient.begin(UDP_PORT); 
                        //timer0_attachInterrupt(ledUpdate);                        
                    }
                    const char* ip_server = ipmessage["ip_server"];
                    String tem_ip((char*)ip_server);
                    /* check number of IP */
                    if(tem_ip.length() > 8)
                    {
                        TRACE Serial.println(tem_ip);
                        webSocket.beginSocketIO(tem_ip, TCP_PORT);
                        webSocket.onEvent(webSocketEvent);
                        isGotIpServer = true;
                    }
                    else goto UDPbroadcast;
                    }
                    //timer0_attachInterrupt(ledUpdate);

                }
            }
        }
        else
        {
            if (g_pingSendCount >= 10 && isServerConnected)
            {
                g_pingSendCount = 0;
                webSocket.disconnect();
                isServerConnected = false;
                isGotIpServer = false;
                TRACE Serial.println("out of ping count");
            }
            if (isServerConnected)
            {
                //timer0_detachInterrupt(); 
                if (!isRegisted)
                {
                    delay(5000);
                    isRegisted = true;
                    //send reg cmd
                    socketCmd += "42[\"wm_register\",";
                    socketCmd += "{";
                    socketCmd += "\"deviceID\":\"";
                    /* Get deviceID from ESP ID chip */
                    socketCmd += ESPID;
                    socketCmd += "\",";
                    socketCmd += "\"operate_state\":";
                    socketCmd += (isStarted ? 2 : 0);
                    socketCmd += "}]";
                    TRACE Serial.println(socketCmd);
                    webSocket.sendTXT(socketCmd);
                    socketCmd = "";
                }
                if (isNewCardFound)
                {
                    isNewCardFound = false;
                    //send card uid
                    socketCmd += "42[\"wm_ask_for_run\",";
                    socketCmd += "{";
                    socketCmd += "\"deviceID\":\"";
                    /* Get deviceID from ESP ID chip */
                    socketCmd += ESPID;
                    socketCmd += "\",";
                    socketCmd += "\"cardID\":";
                    socketCmd += g_cardID;
                    socketCmd += "}]";
                    // send ping
                    TRACE Serial.println(socketCmd);
                    webSocket.sendTXT(socketCmd);
                    socketCmd = "";
                }
                else
                {
                    if (curTime - g_prevSendPingTime >= SEND_PING_CMD)
                    {
                        //timer0_detachInterrupt();
                        g_prevSendPingTime = curTime;
                        // g_pingSendCount++;
                        // webSocket.sendTXT("42[\"messageType\",{\"greeting\":\"hello\"}]");
                        socketCmd += "42[\"wm_ping\",";
                        socketCmd += "{";
                        socketCmd += "\"deviceID\":\"";
                        /* Get deviceID from ESP ID chip */
                        socketCmd += ESPID;
                        socketCmd += "\"}]";
                        // send ping
                        TRACE Serial.println(socketCmd);
                        //timer0_detachInterrupt();
                        digitalWrite(LATCH_PIN, LOW);
                        shiftOut(ledCtrlData);
                        digitalWrite(LATCH_PIN, HIGH); 
                        //webSocket.sendTXT(socketCmd);
                        webSocket.sendTXT("1");
                        //timer0_attachInterrupt(ledUpdate);
                        //TRACE Serial.println(ledCtrlData);
                        socketCmd = "";
                        //timer0_attachInterrupt(ledUpdate);
                    }
                }
                //timer0_attachInterrupt(ledUpdate);
            }
        }
    }

    if (isStarted) // decrease time
    {
        if (g_washTime == 0) // wash finish
        {
            isStarted = false;
            requestWashMachineStop = true;
        }
        if (curTime - g_prevMinuteTime >= ONE_MINUTE)
        {
            g_prevMinuteTime = curTime;
            if (g_washTime)
            {
                digitUpdate(g_washTime);
                g_washTime--;
            }
            if (g_washTime == 0)
            {
                digitUpdate(0);
                isRequested = true;
              //tagID= "A1F32F71F8B";
            }
        }
    }

    if (requestWashMachineStop)
    {
        requestWashMachineStop = false;
        setRelayEnable(false);
        setLedEnable(RUNNING_LED, false);
        if (isWifiConnected && isServerConnected)
        {
            socketCmd += "42[\"wm_stop\",";
            socketCmd += "{";
            socketCmd += "\"deviceID\":\"";
            /* Get deviceID from ESP ID chip */
            socketCmd += ESPID;
            socketCmd += "\",";
            socketCmd += "\"cardID\":\"";
            socketCmd += g_cardID;
            socketCmd += "\"}]";
            // send stop msg
            TRACE Serial.println(socketCmd);
            webSocket.sendTXT(socketCmd);
            socketCmd = "";
        }
    }
}

bool compareCardUID(byte *buffer, byte bufferSize)
{
    byte i = 0;
    bool retVal = true;

    for(i = 0; i < bufferSize; i++)
    {
        if (buffer[i] != card_uid[i])
        {
            retVal = false;
            break;
        }
    }

    return retVal;
}

void setRelayEnable(bool enable)
{
  if (enable)
  { // l l l r b p p p
    ledCtrlData = ledCtrlData & (~(1 << 3));
  }
  else
  {
    ledCtrlData = ledCtrlData | (1 << 3);
  }
}

void setBuzzerOn(uint8_t period)
{

}

// pos = 0 1 2
void setLedEnable(uint8_t pos, bool enable) // need to check pos (lllrrddd)
{
    if (pos > 2)
    {
        return;
    }
    if (enable)
    {
        ledCtrlData = ledCtrlData & (~(1 << (pos + 5)));
    }
    else
    {
        ledCtrlData = (ledCtrlData & (~(1 << (pos + 5)))) | (1 << (pos + 5));
    }
}

void ledUpdate(void)
{
    switch(ledPos){
        case 0: ledData = number[g_led1]; break;
        case 1: ledData = number[g_led2]; break;
        case 2: ledData = number[g_led3]; break;
    }

    // l l l r r p p p

    ledCtrlData = (~(1 << ledPos) & 0x07) | (ledCtrlData & 0xF8);

    if (++ledPos >= 3) ledPos = 0;

    // push out data
    digitalWrite(LATCH_PIN, LOW);
    shiftOut(ledCtrlData);
    shiftOut(ledData);
    digitalWrite(LATCH_PIN, HIGH);
    //timer0_write(ESP.getCycleCount()+6000000);
    //timer1_write(ESP.getCycleCount() + 1000);
}

void digitUpdate(uint32_t number)
{
    g_led1 = (uint8_t)(number / 100);
    g_led2 = (uint8_t)((number / 10) % 10);
    g_led3 = (uint8_t)(number % 10);
}


void shiftOut(byte myDataOut) {
  int i=0;
  int pinState;


  digitalWrite(DATA_PIN, 0);
  digitalWrite(CLK_PIN, 0);

  for (i=7; i>=0; i--)  {
    digitalWrite(CLK_PIN, 0);

    if ( myDataOut & (1<<i) ) {
      pinState= 1;
    }
    else {
      pinState= 0;
    }

    //Sets the pin to HIGH or LOW depending on pinState
    digitalWrite(DATA_PIN, pinState);
    //register shifts bits on upstroke of clock pin
    digitalWrite(CLK_PIN, 1);
    //zero the data pin after shift to prevent bleed through
    digitalWrite(DATA_PIN, 0);
  }

  //stop shifting
  digitalWrite(CLK_PIN, 0);
}

String IpAddress2String(const IPAddress& ipAddress)
{
  return String(ipAddress[0]) + String(".") +\
  String(ipAddress[1]) + String(".") +\
  String(ipAddress[2]) + String(".") +\
  String(ipAddress[3])  ; 
}