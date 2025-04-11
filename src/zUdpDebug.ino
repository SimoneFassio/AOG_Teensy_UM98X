// Buffer configuration
const size_t BUFFER_SIZE = UDP_TX_PACKET_MAX_SIZE;
const unsigned long SEND_INTERVAL = 200; // milliseconds

// Global variables
char buffer[BUFFER_SIZE];
size_t currentPos = 0;
unsigned long lastSendTime = 0;


void udpDebugReceive()
{
#ifdef ARDUINO_TEENSY41
  // When ethernet is not running, return directly. parsePacket() will block when we don't
  if (!Ethernet_running)
  {
    return;
  }

  unsigned int packetLength = Eth_udpDebug.parsePacket();

  if (packetLength>2)
  {
    char packetBuffer[255];
    Eth_udpDebug.read(packetBuffer, packetLength);
    String command = String(packetBuffer);
    if(debugState == EXPERIMENT || send_EXPERIMENT){
      debugPrint("Received command: ");
      debugPrintln(command);
    }
    // Parse command
    int separatorIndex = command.indexOf(':');
    if (separatorIndex != -1) {
        String key = command.substring(0, separatorIndex);
        String valueStr = command.substring(separatorIndex+1);
        
        if (key == "GPS") send_GPS = (valueStr.toInt() == 1);
        else if (key == "EXPERIMENT") send_EXPERIMENT = (valueStr.toInt() == 1);
        else if (key == "KEYA") send_KEYA = (valueStr.toInt() == 1);
        else if (key == "WAS") send_WAS = (valueStr.toInt() == 1);
        else if (key == "INFO") send_INFO = (valueStr.toInt() == 1);
        else if (key == "using2serialGPS") settings.using2serialGPS = (valueStr.toInt() == 1);
        else if (key == "usingWT61") settings.usingWT61 = (valueStr.toInt() == 1);
        else if (key == "interval_INS"){ 
          float old = settings.intervalINS;
          settings.intervalINS = valueStr.toFloat();
          if (old != settings.intervalINS){
            calibrationData.configFlag += 0.01;
            if (calibrationData.configFlag>2)
              calibrationData.configFlag -= 2;
            if (isnan(calibrationData.configFlag))
              calibrationData.configFlag = 0;
            configureUM981();
          }
        }
        else if (key == "useKalmanForSensor") settings.useKalmanForSensor = (valueStr.toInt() == 1);
        else if (key == "minSpeedKalman_m/s") settings.minSpeedKalman = valueStr.toFloat();
        else if (key == "secondsVarBuf") settings.secondsVarianceBuffer = valueStr.toFloat();
        else if (key == "KalmanR") settings.kalmanR = valueStr.toFloat();
        else if (key == "KalmanQ") settings.kalmanQ = valueStr.toFloat();
        if (steerConfig.CytronDriver){   //default tractor
          EEPROM.put(200, settings);
        }
        else {
          EEPROM.put(250, settings);
        }
    }
  }
    // Check time-based send
  if (systick_millis_count - lastSendTime >= SEND_INTERVAL) {
    sendBuffer();
  }
#endif
}


void sendBuffer() {
  if (!Ethernet_running)
  {
    return;
  }
  
  if (currentPos > 0) {
    Eth_udpDebug.beginPacket(Eth_ipDestination, portDebugOUT);
    Eth_udpDebug.write(buffer, currentPos);
    Eth_udpDebug.endPacket();
    currentPos = 0;
    lastSendTime = systick_millis_count;
  }
  else {
    // Reset the buffer if no data to send at startup
    currentPos = 0;
  }
}

void appendToBuffer(const char* data, size_t len) {
  
  if (currentPos + len >= BUFFER_SIZE) {
    sendBuffer();
  }
  
  // Handle data larger than buffer size
  size_t remaining = BUFFER_SIZE - currentPos;
  size_t toCopy = (len > remaining) ? remaining : len;
  
  memcpy(buffer + currentPos, data, toCopy);
  currentPos += toCopy;

  // Check time-based send
  if (systick_millis_count - lastSendTime >= SEND_INTERVAL) {
    sendBuffer();
  }
}

// Overloaded debug functions
void debugPrint(const char* message) {
  appendToBuffer(message, strlen(message));
  Serial.print(message);
  SerialDebug.print(message);
}

void debugPrint(const String &message) {
  appendToBuffer(message.c_str(), message.length());
  Serial.print(message);
  SerialDebug.print(message);
}

void debugPrint(char c) {
  appendToBuffer(&c, 1);
  Serial.print(c);
  SerialDebug.print(c);
}

void debugPrint(int num) {
  char temp[16];
  size_t len = snprintf(temp, sizeof(temp), "%d", num);
  appendToBuffer(temp, len);
  Serial.print(num);
  SerialDebug.print(num);
}

void debugPrint(unsigned int num) {
  char temp[16];
  size_t len = snprintf(temp, sizeof(temp), "%u", num);
  appendToBuffer(temp, len);
  Serial.print(num);
  SerialDebug.print(num);
}

void debugPrint(uint8_t num, int format) {
  char temp[16];
  if(format == HEX) {
    size_t len = snprintf(temp, sizeof(temp), "%02X", num);
    appendToBuffer(temp, len);
    Serial.print(num, HEX);
    SerialDebug.print(num, HEX);
  } else {
    size_t len = snprintf(temp, sizeof(temp), "%u", num);
    appendToBuffer(temp, len);
    Serial.print(num);
    SerialDebug.print(num);
  }
}

void debugPrint(uint8_t num) {
  char temp[16];
  size_t len = snprintf(temp, sizeof(temp), "%" PRIu8, num);
  appendToBuffer(temp, len);
  Serial.print(num);
  SerialDebug.print(num);
}

void debugPrint(uint32_t num) {
  char temp[16];
  size_t len = snprintf(temp, sizeof(temp), "%" PRIu32, num);
  appendToBuffer(temp, len);
  Serial.print(num);
  SerialDebug.print(num);
}

void debugPrint(int32_t num) {
  char temp[16];
  size_t len = snprintf(temp, sizeof(temp), "%" PRId32, num);
  appendToBuffer(temp, len);
  Serial.print(num);
  SerialDebug.print(num);
}

void debugPrint(float num) {
  int precision = 2;
  char temp[32];
  size_t len = snprintf(temp, sizeof(temp), "%.*f", precision, num);
  appendToBuffer(temp, len);
  Serial.print(num, precision);
  SerialDebug.print(num, precision);
}

// void debugPrint(float num, int precision = 2) {
//   char temp[32];
//   size_t len = snprintf(temp, sizeof(temp), "%.*f", precision, num);
//   appendToBuffer(temp, len);
//   Serial.print(num, precision);
//   SerialDebug.print(num, precision);
// }

void debugPrint(double num, int precision = 2) {
  char temp[32];
  size_t len = snprintf(temp, sizeof(temp), "%.*f", precision, num);
  appendToBuffer(temp, len);
  Serial.print(num, precision);
  SerialDebug.print(num, precision);
}

void debugPrint(const IPAddress &ip) {
  char temp[16];
  size_t len = snprintf(temp, sizeof(temp), "%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);
  appendToBuffer(temp, len);
  Serial.print(temp);
  SerialDebug.print(temp);
}



// println overloads

// Debug println implementations
void debugPrintln(const char* message) {
  debugPrint(message);
  debugPrint("\n");
}

void debugPrintln(const String &message) {
  debugPrint(message);
  debugPrint("\n");
}

void debugPrintln(char c) {
  debugPrint(c);
  debugPrint("\n");
}

void debugPrintln(int num) {
  debugPrint(num);
  debugPrint("\n");
}

void debugPrintln(unsigned int num) {
  debugPrint(num);
  debugPrint("\n");
}

void debugPrintln(uint8_t num, int format) {
  debugPrint(num, format);
  debugPrint("\n");
}

void debugPrintln(uint8_t num) {
  debugPrint(num);
  debugPrint("\n");
}

void debugPrintln(uint32_t num) {
  debugPrint(num);
  debugPrint("\n");
}

void debugPrintln(int32_t num) {
  debugPrint(num);
  debugPrint("\n");
}

void debugPrintln(float num, int precision) {
  debugPrint(num, precision);
  debugPrint("\n");
}

void debugPrintln(float num) {
  debugPrint(num, 2);
  debugPrint("\n");
}

void debugPrintln(double num, int precision) {
  debugPrint(num, precision);
  debugPrint("\n");
}

// Default precision version for double
void debugPrintln(double num) {
  debugPrint(num, 2);  // Match your float default precision
  debugPrint("\n");
}

void debugPrintln() {
  debugPrint("\n");
}

void debugPrintln(const IPAddress &ip) {
  debugPrint(ip);
  debugPrint("\n");
}