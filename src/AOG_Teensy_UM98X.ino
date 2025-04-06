/* Copywrite 2024 chriskinal
This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as
published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied 
warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with this program. If not, see <https://www.gnu.org/licenses/>.
Forked from https://github.com/AgHardware/Boards/blob/main/TeensyModules/AIO%20Standard%20v4/Firmware/Autosteer_gps_teensy_v4/Autosteer_gps_teensy_v4.ino
*/

/*
Updated by Simone Fassio, 2025
This code is a fork of the original code, which is licensed under the GNU General Public License v3.0.
My main contribution are: 
- the addition of support for the UM981 GPS module, which is a different GPS module than the one used in the original code.
- implementation of a Kalman filter for estimating the wheel angle using an encoder instead of the WAS.
*/



// UM982 Connection plan:
// Teensy Serial 7 RX (28) to F9P Position receiver TX1 (Position data)
// Teensy Serial 7 TX (29) to F9P Position receiver RX1 (RTCM data for RTK)
// Keya bits from https://github.com/lansalot/AgOpenGPS_Boards/tree/KeyaV2/TeensyModules/V4.1/Firmware/Autosteer_gps_teensy_v4_1

#include "zNMEAParser.h"
#include <Wire.h>
#include "BNO08x_AOG.h"
#include <FlexCAN_T4.h>
#include <REG.h>
#include <wit_c_sdk.h>
#include <SimpleKalmanFilter.h>
// Ethernet Options (Teensy 4.1 Only)
#ifdef ARDUINO_TEENSY41
#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>
#include <InternalTemperature.h>
#endif // ARDUINO_TEENSY41

/************************* User Settings *************************/
//#define PCB_VERSION_0_1  // PCB version 0.1
#define PCB_VERSION_1_0  // PCB version 1.0

bool udpPassthrough = false;  // False = GPS neeeds to send GGA, VTG & HPR messages. True = GPS needs to send KSXT messages only.
bool makeOGI = true;          // Set to true to make PAOGI messages. Else PANDA message will be made.
const bool invertRoll = true; // Invert Roll in BNO

struct Config {
    bool using2serialGPS=true;
    bool usingWT61=false;
    float intervalINS=0.1; // 0.1 or 0.05 -> 10 or 20 Hz
    bool useKalmanForSensor=true;
    float minSpeedKalman=0.5; //m /s
    float secondsVarianceBuffer=3; // pay attention to max varianceBuffer len in zKalmanKeya
    float kalmanR=0.1;
    float kalmanQ=0.0001;
};

Config settings;

#ifdef PCB_VERSION_0_1
// Serial Ports
#define SerialRTK Serial2             // RTK radio
#define SerialWT61 Serial3            // IMU
#define SerialGPS2 Serial4            // Main postion receiver (INS)
#define SerialGPS Serial7           // Main postion receiver (GGA, VTG)
#define SerialDebug Serial8         // Debug TX only

// Status LED's   
#define GGAReceivedLED 12        // blink if GGA received, ON if INS OK, OFF no GGA     blue
#define DEBUG_LED 13             // ON if debugState > SETUP                          red on board
#define AUTOSTEER_ACTIVE_LED 10  // blink if hello from AOG, ON if steering,          red
#define CAN_ACTIVE_LED 9         // ON if keya heartbeat,                             yellow
#define DEBUG_PIN 37             //button

#endif


#ifdef PCB_VERSION_1_0
// Serial Ports
#define SerialRTK Serial2             // RTK radio
#define SerialWT61 Serial1            // IMU
#define SerialGPS2 Serial4            // Main postion receiver (INS)
#define SerialGPS Serial3           // Main postion receiver (GGA, VTG)
#define SerialDebug Serial8         // Debug TX only

// Status LED's   
#define GGAReceivedLED 38        // blink if GGA received, ON if INS OK, OFF no GGA     red
#define DEBUG_LED 13             // ON if debugState > SETUP                          red on board
#define CAN_ACTIVE_LED 37        // ON if keya heartbeat, blink if hello from AOG but no keya      green
#define DEBUG_PIN 3             //button

#endif

const int32_t baudAOG = 115200;       // USB connection speed
const int32_t baudGPS = 460800;       // UM982 connection speed
const int32_t baudRTK = 9600;         // most are using Xbee radios with default of 115200

// Send data to AgIO via usb
bool sendUSB = false;

struct ConfigIP
{
  uint8_t ipOne = 192;
  uint8_t ipTwo = 168;
  uint8_t ipThree = 5;
};

/************************* End User Settings *********************/
#define ImuWire Wire // SCL=19:A5 SDA=18:A4

// Keya Support
// CRX1/CTX1 on Teensy are CAN1 on Tony's board
// CRX2/CTX2 on Teensy are CAN2 on AIO board, CAN2 on Tony's board
// CRX3/CTX3 on Teensy are CAN1 on AIO board, CAN3 on Tony's board
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_256> Keya_Bus;
float KeyaCurrentSensorReading = 0;
float KeyaCurrentRapport = 0;
float KeyaCurrentRapportSmooth = 0;

bool keyaDetected = false;

#define REPORT_INTERVAL 20  // BNO report time, we want to keep reading it quick & offen. Its not timmed to anything just give constant data.
uint32_t READ_BNO_TIME = 0; // Used stop BNO data pile up (This version is without resetting BNO everytime)

uint8_t debug_pin = DEBUG_PIN;

uint32_t gpsReadyTime = 0;       // Used for GGA timeout      
uint32_t KeyaBeatTime = 0;       // Used for Keya timeout

void errorHandler();
void GGA_Handler();
void VTG_Handler();
void HPR_Handler();
void INS_Handler();
void autosteerSetup();
void EthernetStart();
void udpNtrip();
void BuildNmea();
void readBNO();
void autosteerLoop();
void ReceiveUdp();
void checkUM982();
void configureUM982();
void checkUM981();
void configureUM981();
void imuSetup();
void passthroughSerial();
void LedSetup();
void getKeyaInfo();
void angleStimeUpdate();
void tryKeyaCommand();

ConfigIP networkAddress; // 3 bytes

// IP & MAC address of this module of this module
byte Eth_myip[4] = {0, 0, 0, 0}; // This is now set via AgIO
byte mac[] = {0x00, 0x00, 0x56, 0x00, 0x00, 0x78};

unsigned int portMy = 5120;           // port of this module
unsigned int AOGNtripPort = 2233;     // port NTRIP data from AOG comes in
unsigned int AOGAutoSteerPort = 8888; // port Autosteer data from AOG comes in
unsigned int portDestination = 9999;  // Port of AOG that listens
unsigned int portDebugOUT = 6969;   // Port debug
unsigned int portDebugIN = 9696;    // Port debug
unsigned int portPlot = 6968;       // Port plot
char Eth_NTRIP_packetBuffer[512];     // buffer for receiving ntrip data

// An EthernetUDP instance to let us send and receive packets over UDP
EthernetUDP Eth_udpPAOGI;     // Out port 5544
EthernetUDP Eth_udpNtrip;     // In port 2233
EthernetUDP Eth_udpAutoSteer; // In & Out Port 8888
EthernetUDP Eth_udpDebug;     // In 9696 & Out Port 6969

IPAddress Eth_ipDestination;

byte CK_A = 0;

bool gotCR = false;
bool gotLF = false;
bool gotDollar = false;
char msgBuf[254];
int msgBufLen = 0;

uint8_t dualReadyINS = 0;
uint8_t numReadyINS = settings.intervalINS < 0.07 ? 2 : 1;

// booleans to see if we are using BNO08x
bool useBNO08x = false;

// BNO08x address variables to check where it is
const uint8_t bno08xAddresses[] = {0x4A, 0x4B};
const int16_t nrBNO08xAdresses = sizeof(bno08xAddresses) / sizeof(bno08xAddresses[0]);
uint8_t bno08xAddress;
BNO080 bno08x;

byte ackPacket[72] = {0xB5, 0x62, 0x01, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

constexpr int serial_buffer_size = 2040;
uint8_t GPSrxbuffer[serial_buffer_size];  // Extra serial rx buffer
uint8_t GPStxbuffer[serial_buffer_size];  // Extra serial tx buffer
uint8_t RTKrxbuffer[serial_buffer_size];  // Extra serial rx buffer
const int tmp_serial_buffer_size = 2048;
uint8_t tmpGPSrxbuffer[tmp_serial_buffer_size]; // Temp serial rx buffer for detecting / configuring the UM982
uint8_t tmpGPStxbuffer[tmp_serial_buffer_size]; // Temp serial tx buffer for detecting / configuring the UM982

// Baudrates for detecting UM982 receiver
uint32_t baudrates[]{
    460800,
    115200};
const uint32_t nrBaudrates = sizeof(baudrates) / sizeof(baudrates[0]);

// Bools for detecting / configuring the UM982
bool gotUM982 = false;
bool setUM982 = false;
bool gotUM981 = false;
bool setUM981 = false;
bool usingUM982 = false;

/* A parser is declared with 4 handlers at most */
NMEAParser<3> parser;

bool blink = false;

bool Autosteer_running = false; // Auto set off in autosteer setup
bool Ethernet_running = false; // Auto set on in ethernet setup

float tempWT;

enum debugList {
  SETUP,
  EXPERIMENT,
  ROLL,
  WAS,
  GPS,
  KEYA,
  SWITCH,
  UDP,
  STATE_INFO
};

enum debugList debugState = SETUP;

bool send_GPS = false;
bool send_EXPERIMENT = false;
bool send_KEYA = false;
bool send_WAS = false;
bool send_INFO = false;

int8_t debugButton = 1;
uint32_t debugTime = 0;
unsigned long lastPlotTime = 0;
const unsigned long plotInterval = 100;  // ms

struct CalibrationData {
  float wheelBase = 2.4;
  float IMUtoANTx = 0.0;
  float IMUtoANTy = 0.5;
  float IMUtoANTz = 1.5;
  float INSx = 0.0;
  float INSy = -0.5;
  float INSz = -1.5;
  float INSanglex = 0.0;
  float INSangley = 0.0;
  float INSanglez = 0.0;
  float configFlag = 0.01;
};

CalibrationData calibrationData;

// Setup procedure ---------------------------------------------------------------------------------------------------------------
void setup()
{
  LedSetup();

  delay(10);
  Serial.begin(baudAOG);
  SerialDebug.begin(baudAOG);
  delay(10);

  debugPrintln("Start setup");
  debugPrintln();


  if (settings.usingWT61)
    setupWT61();


  debugPrintln("\r\nStarting AutoSteer...");
  autosteerSetup();


  debugPrintln("\r\nStarting Ethernet...");
  EthernetStart();

  checkUM981();

  if (gotUM981)
    configureUM981();
  else {
    checkUM982();
    if (gotUM982)
      configureUM982();
    usingUM982 = true;
  }

  // the dash means wildcard

  parser.setErrorHandler(errorHandler);
  parser.addHandler("G-GGA", GGA_Handler);
  parser.addHandler("G-VTG", VTG_Handler);
  if(usingUM982)
    parser.addHandler("G-HPR", HPR_Handler);


  SerialGPS2.begin(baudGPS);
  SerialGPS.begin(baudGPS);
  SerialGPS.addMemoryForRead(GPSrxbuffer, serial_buffer_size);
  SerialGPS.addMemoryForWrite(GPStxbuffer, serial_buffer_size);

  delay(10);
  SerialRTK.begin(baudRTK);
  SerialRTK.addMemoryForRead(RTKrxbuffer, serial_buffer_size);

  debugPrintln("Serial, SerialRTK, SerialGPS initialized");

  //debugPrintln("\r\nStarting BNO085...");
  //imuSetup();

  useBNO08x=false; //*********************************

  delay(100);
  //debugPrint("useBNO08x = ");
  //debugPrintln(useBNO08x);

  // Keya support
  CAN_Setup();


  debugPrintln("\r\nEnd setup, waiting for GPS...\r\n");

}

void loop()
{
  // Keya support
  tryKeyaCommand();
  KeyaBus_Receive();
  udpDebugReceive();

  // WT61 IMU receive
  if (settings.usingWT61)
    loopWT61();

  // Read incoming nmea from GPS
  if (SerialGPS.available())
  {
    if (udpPassthrough)
    {
      passthroughSerial();
    }
    else if (settings.using2serialGPS || usingUM982)
    {
      char c = SerialGPS.read();
      if(debugState == GPS || send_GPS)
        debugPrint(c);
      parser << c;
    }
    else
    {
      readSerialIns(SerialGPS.read());
    }
  }

  if(settings.using2serialGPS && SerialGPS2.available())
  {
    readSerialIns(SerialGPS2.read());
  }

  udpNtrip();

  // If both dual messages are ready, send to AgOpen
  if (dualReadyINS >= numReadyINS)
  {
    imuHandler();
    BuildNmea();
    dualReadyINS = 0;
  }

  // Read BNO
  // if ((systick_millis_count - READ_BNO_TIME) > REPORT_INTERVAL && useBNO08x)
  // {
  //   READ_BNO_TIME = systick_millis_count;
  //   readBNO();
  // }

  //turn off LED if GGa timeout 3 sec
  if (systick_millis_count - gpsReadyTime > 3000)
    digitalWrite(GGAReceivedLED, LOW);

  if(systick_millis_count - KeyaBeatTime > 5000){
    if(!Autosteer_running)
      digitalWrite(CAN_ACTIVE_LED, LOW);
    keyaDetected = false;
  }

  if (Autosteer_running){
    autosteerLoop();
  }
  else{
    ReceiveUdp();

    #ifdef PCB_VERSION_0_1
    digitalWrite(AUTOSTEER_ACTIVE_LED, 0);
    #endif

    #ifdef PCB_VERSION_1_0
    if(!keyaDetected)
      digitalWrite(CAN_ACTIVE_LED, 0);
    #endif
  }

   // Send plot data every 100ms
  if (millis() - lastPlotTime >= plotInterval && send_WAS) {
    sendPlotData();
    lastPlotTime = millis();
  }

  debugLoop();
} // End Loop
//**************************************************************************
