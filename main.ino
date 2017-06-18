#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <SPI.h>
#include <stdint.h>
#include <WebSocketsClient.h>
#include <Hash.h>
#include <ArduinoJson.h>
#include "FS.h"
#include "MFRC522.h"

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
#define CARD_SCAN_INTERLVAL     100 // 0,5 s
#define ONE_MINUTE              60000 //60000 // 60k ms
#define WASH_TIME               35 // 35 min
#define SEND_BROADCAST_CMD      1000
#define SEND_PING_CMD           1000

#define UDP_PORT                6969
#define TCP_PORT                9696

#define WIFI_LED                0
#define SEVER_LED               1
#define RUNNING_LED             2

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
bool isRequested = false;
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
uint32_t deviceID = 0;
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

void setup() {
    size_t size;
    StaticJsonBuffer<200> jsonBuffer;

    Serial.begin(9600);    // Initialize serial communications
    delay(100);

    Serial.println("\r\nInitialize....");

    SPI.begin();           // Init SPI bus
    mfrc522.PCD_Init();    // Init MFRC522
    mfrc522.PCD_SetAntennaGain(mfrc522.RxGain_max);
    // Init pin
    pinMode(DATA_PIN, OUTPUT);
    pinMode(CLK_PIN, OUTPUT);
    pinMode(LATCH_PIN, OUTPUT);
    pinMode(BTN_PIN, INPUT_PULLUP);
    //attachInterrupt(digitalPinToInterrupt(BTN_PIN), btnHandler, FALLING );

    digitalWrite(DATA_PIN, LOW);
    digitalWrite(CLK_PIN, LOW);
    digitalWrite(LATCH_PIN, LOW);

    if (!SPIFFS.begin()) {
        Serial.println("Failed to mount file system");
    }

    // load config info
    File configFile = SPIFFS.open("/config.json", "r");
    if (!configFile) {
        Serial.println("Failed to open config file");
    }
    else
    {
        size = configFile.size();
        if (size > 1024) {
            Serial.println("Config file size is too large");
        }
        else
        {
            // Allocate a buffer to store contents of the file.
            std::unique_ptr<char[]> buf(new char[size]);

            configFile.readBytes(buf.get(), size);

            JsonObject& json = jsonBuffer.parseObject(buf.get());

            if (!json.success()) {
                Serial.println("Failed to parse config file");
            }

            ssid = json["ssid"];
            pass = json["pass"];
            deviceID = json["deviceID"];

            if (String(ssid) == "test@nopass" && String(pass) == "nopass")
            {
                // start smart config
                Serial.println("Enter config mode");
                isEnteredConfig = true;
                WiFi.mode(WIFI_AP_STA);
                delay(500);
                WiFi.beginSmartConfig();
            }

            Serial.println(String(ssid));
            Serial.println(String(pass));
            Serial.println(deviceID);
        }
    }

    if (!isEnteredConfig)
    {
        WiFi.begin(ssid, pass);
        udpClient.begin(UDP_PORT);
    }

    // close file
    configFile.close();

    setLedEnable(RUNNING_LED, false);
    setLedEnable(WIFI_LED, false);
    setLedEnable(SEVER_LED, false);
    setRelayEnable(false);

    delay(5000);

    timer0_isr_init();
    timer0_attachInterrupt(ledUpdate);
    timer0_write(ESP.getCycleCount()+60000);

    Serial.println(F("Ready!"));
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
                Serial.printf("[WSc] Disconnected!\n");
                isServerConnected = false;
                setLedEnable(SEVER_LED, false);
                isGotIpServer = false;
                isRegisted = false;
                udpClient.begin(UDP_PORT);
                webSocket.disconnect();
            }
            break;
        case WStype_CONNECTED:
            {
                Serial.printf("[WSc] Connected to url: %s\n",  payload);
                setLedEnable(SEVER_LED, true);
                isServerConnected = true;
                webSocket.sendTXT("5");
            }
            break;
        case WStype_TEXT:
            {
                if (length > 2)
                {
                    // 42["response","{\"code\":0}"]
                    if (payload[0] == '4' && payload[1] == '2')
                    {
                        String resData((const char*)payload);
                        Serial.println(resData);
                        int idx = resData.indexOf("code");
                        if (idx != -1)
                        {
                            int idx1 = resData.indexOf(':');
                            if (idx1 != -1)
                            {
                                int responseCode = payload[idx1 + 1] - 48;
                                if (responseCode == 2 && isRequested)
                                {
                                    isRequested = false;
                                    isStarted = true;
                                    g_washTime = WASH_TIME;
                                    setLedEnable(RUNNING_LED, true);
                                    setRelayEnable(true);
                                    Serial.println("exec");
                                }
                            }
                        }
                    }
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

    if (WiFi.status() == WL_CONNECTED && isWifiConnected == false)
    {
        if (isEnteredConfig)
        {
            WiFi.smartConfigDone();
            isEnteredConfig = false;
            // save new data to config file
            File wf = SPIFFS.open("/config.json", "w");
            if (!wf) {
                Serial.println("Failed to open config file to write");
            }
            else
            {
                StaticJsonBuffer<200> jsonBuffer;
                JsonObject& json = jsonBuffer.createObject();
                json["ssid"] = WiFi.SSID();
                json["pass"] = WiFi.psk();
                json["deviceID"] = deviceID;

                Serial.println(String((const char*)json["ssid"]));
                Serial.println(String((const char*)json["pass"]));

                json.printTo(wf);

                wf.flush();
                wf.close();

                SPIFFS.end();
            }
            // system reboot
            ESP.restart();
        }
        if (!isWifiConnected)
        {
            broadcastIp = ~WiFi.subnetMask() | WiFi.gatewayIP();
            localIP = WiFi.localIP();
            isWifiConnected = true;
        }
        setLedEnable(WIFI_LED, true);
        Serial.println("Connected");
    }
    else if (WiFi.status() != WL_CONNECTED && isWifiConnected == true)
    {
        Serial.println("Disconnected");
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
        webSocket.loop();
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
                // Serial.print(mfrc522.uid.uidByte[i] < 0x10 ? " 0" : " ");
                // Serial.print(mfrc522.uid.uidByte[i], HEX);
            }
            if (tmpCardId != g_cardID) // new card found
            {
                if (isStarted)
                {
                    isStarted = false;
                    requestWashMachineStop = true;
                }
                g_cardID = tmpCardId;
                isNewCardFound = true;
            }
            // Serial.println();
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
                        requestWashMachineStop = true;
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

    if (isWifiConnected)
    {
        if (!isGotIpServer)
        {
            if (curTime - g_prevSendBroadcastCmdTime >= SEND_BROADCAST_CMD)
            {
                String broadcastMsg;
                g_prevSendBroadcastCmdTime = curTime;
                Serial.println("broadcast msg");
                // send broadcast cmd
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
            }
            else
            {
                packetSize = udpClient.parsePacket();

                if (packetSize)
                {
                    // read the packet into packetBufffer
                    int len = udpClient.read(packetBuffer, 255);
                    if (len > 0) {
                      packetBuffer[len] = 0;
                    }
                    remoteIp = udpClient.remoteIP();
                    isGotIpServer = true;
                    Serial.println(remoteIp);
                    // if got ip server
                    {
                        String ipAddr;
                        ipAddr += remoteIp[0];
                        ipAddr += '.';
                        ipAddr += remoteIp[1];
                        ipAddr += '.';
                        ipAddr += remoteIp[2];
                        ipAddr += '.';
                        ipAddr += remoteIp[3];
                        udpClient.flush();
                        udpClient.stopAll();
                        webSocket.beginSocketIO(ipAddr, TCP_PORT);
                        webSocket.onEvent(webSocketEvent);
                    }
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
                Serial.println("out of ping count");
            }
            if (isServerConnected)
            {
                if (!isRegisted)
                {
                    isRegisted = true;
                    //send reg cmd
                    socketCmd += "42[\"wm_register\",";
                    socketCmd += "{";
                    socketCmd += "\"deviceID\":";
                    socketCmd += deviceID;
                    socketCmd += ",";
                    socketCmd += "\"operate_state\":";
                    socketCmd += (isStarted ? 2 : 0);
                    socketCmd += "}]";
                    Serial.println(socketCmd);
                    webSocket.sendTXT(socketCmd);
                    socketCmd = "";
                }
                if (isNewCardFound)
                {
                    isNewCardFound = false;
                    //send card uid
                    socketCmd += "42[\"wm_ask_for_run\",";
                    socketCmd += "{";
                    socketCmd += "\"deviceID\":";
                    socketCmd += deviceID;
                    socketCmd += ",";
                    socketCmd += "\"cardID\":";
                    socketCmd += g_cardID;
                    socketCmd += "}]";
                    // send ping
                    Serial.println(socketCmd);
                    webSocket.sendTXT(socketCmd);
                    isRequested = true;
                    socketCmd = "";
                }
                else
                {
                    if (curTime - g_prevSendPingTime >= SEND_PING_CMD)
                    {
                        g_prevSendPingTime = curTime;
                        // g_pingSendCount++;
                        // webSocket.sendTXT("42[\"messageType\",{\"greeting\":\"hello\"}]");
                        socketCmd += "42[\"wm_ping\",";
                        socketCmd += "{";
                        socketCmd += "\"deviceID\":";
                        socketCmd += deviceID;
                        socketCmd += "}]";
                        // send ping
                        // Serial.println(socketCmd);
                        webSocket.sendTXT(socketCmd);
                        // Serial.println(ledCtrlData);
                        socketCmd = "";
                    }
                }
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
            if (g_washTime)
            {
                g_washTime--;
                g_prevMinuteTime = curTime;
                g_led1 = (uint8_t)(g_washTime / 100);
                g_led2 = (uint8_t)((g_washTime / 10) % 10);
                g_led3 = (uint8_t)(g_washTime % 10);
            }
            if (g_washTime == 0)
            {
                g_led1 = 0;
                g_led2 = 0;
                g_led3 = 0;
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
            socketCmd += "\"deviceID\":";
            socketCmd += deviceID;
            socketCmd += ",";
            socketCmd += "\"cardID\":";
            socketCmd += g_cardID;
            socketCmd += "}]";
            // send stop msg
            Serial.println(socketCmd);
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
    timer0_write(ESP.getCycleCount()+600000);
    // Serial.println("in led update");
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
