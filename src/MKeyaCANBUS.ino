// KeyaCANBUS
/*
"Barrowed" Keya code from Matt Elias @ https://github.com/m-elias/AgOpenGPS_Boards/tree/575R-Keya/TeensyModules/V4.1"
*/

#define lowByte(w) ((uint8_t)((w) & 0xFF))
#define highByte(w) ((uint8_t)((w) >> 8))

// templates for commands with matching responses, only need first 4 bytes
uint8_t keyaDisableCommand[] = {0x23, 0x0C, 0x20, 0x01};
uint8_t keyaDisableResponse[] = {0x60, 0x0C, 0x20, 0x00};

uint8_t keyaEnableCommand[] = {0x23, 0x0D, 0x20, 0x01};
uint8_t keyaEnableResponse[] = {0x60, 0x0D, 0x20, 0x00};

uint8_t keyaSpeedCommand[] = {0x23, 0x00, 0x20, 0x01};
uint8_t keyaSpeedResponse[] = {0x60, 0x00, 0x20, 0x00};

uint8_t keyaCurrentQuery[] = {0x40, 0x00, 0x21, 0x01};
uint8_t keyaCurrentResponse[] = {0x60, 0x00, 0x21, 0x01};

uint8_t keyaFaultQuery[] = {0x40, 0x12, 0x21, 0x01};
uint8_t keyaFaultResponse[] = {0x60, 0x12, 0x21, 0x01};

uint8_t keyaVoltageQuery[] = {0x40, 0x0D, 0x21, 0x02};
uint8_t keyaVoltageResponse[] = {0x60, 0x0D, 0x21, 0x02};

uint8_t keyaTemperatureQuery[] = {0x40, 0x0F, 0x21, 0x01};
uint8_t keyaTemperatureResponse[] = {0x60, 0x0F, 0x21, 0x01};

uint8_t keyaVersionQuery[] = {0x40, 0x01, 0x11, 0x11};
uint8_t keyaVersionResponse[] = {0x60, 0x01, 0x11, 0x11};

uint8_t keyaEncoderResponse[] = {0x60, 0x04, 0x21, 0x01};

uint8_t keyaEncoderSpeedResponse[] = {0x60, 0x03, 0x21, 0x01};

uint64_t KeyaID = 0x06000001; // 0x01 is default ID

bool lnNeeded = false;
uint32_t hbTime;
uint32_t keyaTime;
uint32_t keyaCommandTime=0;
uint8_t keyaCommandState=0;
bool keyaMotorStatus = false;
int16_t actualSpeed = 0;

int32_t keyaEncoderValue = 0;
int32_t keyaEncoderValueOld = 0;
int32_t keyaEncoderValueFreeze = 0;
int32_t keyaEncoderVirtual = 0;
int32_t keyaEncoderFinalOld = 0;
int32_t keyaEncoderDiff = 0;
int8_t keyaDir = 0;
uint8_t keyaState = 0; // 0 -> 4


void CAN_Setup()
{
  Keya_Bus.begin();
  Keya_Bus.setBaudRate(250000); // for official Keya/jnky motor
  // Keya_Bus.setBaudRate(500000);  // for identical motor from JinanLanJiu store https://www.aliexpress.com/item/1005005364248561.html
  delay(100);
  debugPrint("Initialised Keya CANBUS @ ");
  debugPrint(Keya_Bus.getBaudRate());
  debugPrintln("bps");
}

bool isPatternMatch(const CAN_message_t &message, const uint8_t *pattern, size_t patternSize)
{
  return memcmp(message.buf, pattern, patternSize) == 0;
}

void printIdAndReply(uint32_t id, uint8_t buf[8])
{
  if(debugState==KEYA){
    debugPrint(systick_millis_count);
    debugPrint(" -> ");
    debugPrint(id, HEX);
    debugPrint(" <> ");
    for (byte i = 0; i < 8; i++)
    {
      if (buf[i] < 16)
        debugPrint("0");
      debugPrint(buf[i], HEX);
      if (i < 7)
        debugPrint(":");
    }
    lnNeeded = true;
  }
}

// only issue one query at a time, wait for respone
void keyaCommand(uint8_t command[])
{
  if (keyaDetected)
  {
    CAN_message_t KeyaBusSendData;
    KeyaBusSendData.id = KeyaID;
    KeyaBusSendData.flags.extended = true;
    KeyaBusSendData.len = 8;
    memcpy(KeyaBusSendData.buf, command, 4);
    Keya_Bus.write(KeyaBusSendData);
  }
}

void tryKeyaCommand()
{
  if(keyaDetected)
  {
    uint8_t remain = (systick_millis_count - keyaCommandTime)%30;
    if(remain < 10 && keyaCommandState==0){
      keyaCommand(keyaEncoderSpeedQuery);
      keyaCommandState++;
    }
    else if(remain > 10 && keyaCommandState==1){
      keyaCommand(keyaCurrentQuery);
      keyaCommandState++;
    }
    else if(remain > 20 && keyaCommandState==2){
      keyaCommand(keyaEncoderQuery);
      keyaCommandState=0;
    }
  }
}

void SteerKeya(int steerSpeed)
{
  if (steerSpeed == 0)
  {
    keyaCommand(keyaDisableCommand);
    if (debugState == KEYA)
      debugPrintln("steerSpeed zero - disabling");
    return; // don't need to go any further, if we're disabling, we're disabling
  }

  if (keyaDetected)
  {
    actualSpeed = map(steerSpeed, -255, 255, -995, 995);
    if (debugState == KEYA)
      debugPrintln("told to steer, with " + String(steerSpeed) + " so....");
    if (debugState == KEYA)
      debugPrintln("I converted that to speed " + String(actualSpeed));

    CAN_message_t KeyaBusSendData;
    KeyaBusSendData.id = KeyaID;
    KeyaBusSendData.flags.extended = true;
    KeyaBusSendData.len = 8;
    memcpy(KeyaBusSendData.buf, keyaSpeedCommand, 4);
    if (steerSpeed < 0)
    {
      KeyaBusSendData.buf[4] = highByte(actualSpeed);
      KeyaBusSendData.buf[5] = lowByte(actualSpeed);
      KeyaBusSendData.buf[6] = 0xff;
      KeyaBusSendData.buf[7] = 0xff;
      if (debugState == KEYA)
        debugPrintln("pwmDrive < zero - clockwise - steerSpeed " + String(steerSpeed));
    }
    else
    {
      KeyaBusSendData.buf[4] = highByte(actualSpeed);
      KeyaBusSendData.buf[5] = lowByte(actualSpeed);
      KeyaBusSendData.buf[6] = 0x00;
      KeyaBusSendData.buf[7] = 0x00;
      if (debugState == KEYA)
        debugPrintln("pwmDrive > zero - anticlock-clockwise - steerSpeed " + String(steerSpeed));
    }
    Keya_Bus.write(KeyaBusSendData);
    keyaCommand(keyaEnableCommand);
  }
}

void KeyaBus_Receive()
{
  CAN_message_t KeyaBusReceiveData;
  if (Keya_Bus.read(KeyaBusReceiveData))
  {
    // parse the different message types

    // heartbeat 00:07:00:00:00:00:00:[ID]
    if (KeyaBusReceiveData.id == 0x07000001)
    {
      KeyaBeatTime = systick_millis_count;
      if (!keyaDetected)
      {
        debugPrintln("Keya heartbeat detected! Enabling Keya canbus & using reported motor current for disengage");
        keyaDetected = true;
        // keyaCommand(keyaVersionQuery);
        digitalWrite(CAN_ACTIVE_LED, 1);
      }
      // 0-1 - Cumulative value of angle (360 def / circle)
      // 2-3 - Motor speed, signed int eg -500 or 500
      // 4-5 - Motor current, with "symbol" ? Signed I think that means, but it does appear to be a crap int. 1, 2 for 1, 2 amps etc
      //		is that accurate enough for us?
      // 6-7 - Control_Close (error code)
      // TODO Yeah, if we ever see something here, fire off a disable, refuse to engage autosteer or..?
      uint32_t time = millis();
      keyaMotorStatus = !bitRead(KeyaBusReceiveData.buf[7], 0);
      if(debugState == KEYA){
        debugPrint(time);
        debugPrint(" ");
        debugPrint(time - hbTime);
        debugPrint(" ");
        hbTime = time;
        printIdAndReply(KeyaBusReceiveData.id, KeyaBusReceiveData.buf);
        debugPrint(" HB ");

        // calc speed
        debugPrint(KeyaBusReceiveData.buf[2]);
        debugPrint(":");
        debugPrint(KeyaBusReceiveData.buf[3]);
        debugPrint("=");
        if (KeyaBusReceiveData.buf[2] == 0xFF)
        {
          debugPrint("-");
          debugPrint(255 - KeyaBusReceiveData.buf[3]);
        }
        else
        {
          debugPrint(KeyaBusReceiveData.buf[3]);
        }
        debugPrint(" ");

        // calc current
        debugPrint(KeyaBusReceiveData.buf[4]);
        debugPrint(":");
        debugPrint(KeyaBusReceiveData.buf[5]);
        debugPrint("=");
        if (KeyaBusReceiveData.buf[4] == 0xFF)
        {
          debugPrint("-");
          debugPrint(255 - KeyaBusReceiveData.buf[5]);
          // KeyaCurrentSensorReading = (255 - KeyaBusReceiveData.buf[5]) * 20;  // use other motor current query data
        }
        else
        {
          debugPrint(KeyaBusReceiveData.buf[5]);
          // KeyaCurrentSensorReading = KeyaBusReceiveData.buf[5] * 20;
        }
        debugPrint(" ");

        // print error status
        debugPrint(KeyaBusReceiveData.buf[6]);
        debugPrint(":");
        debugPrint(KeyaBusReceiveData.buf[7]);
        debugPrint(" "); // debugPrint(KeyaCurrentSensorReading);
        debugPrint("\r\nmotor status ");
        debugPrint(keyaMotorStatus);
      }

      // check if there's any motor diag/error data and parse it
      if (KeyaBusReceiveData.buf[7] != 0)
      {

        // motor disabled bit
        if (bitRead(KeyaBusReceiveData.buf[7], 0))
        {
          if (steerSwitch == 0 && keyaMotorStatus == 1)
          {            
            keyaMotorStatus = !bitRead(KeyaBusReceiveData.buf[7], 0);  //necessario ??  #######################################
            if(debugState == KEYA){
              debugPrint("\r\nMotor disabled");
              debugPrint(" - set AS off");
            }
            steerSwitch = 1; // turn off AS if motor's internal shutdown triggers
            currentState = 1;
            previous = 0;
          }
        }
        else if (bitRead(KeyaBusReceiveData.buf[7], 1))
        {
          debugPrintln("\r\nOver voltage");
        }
        else if (bitRead(KeyaBusReceiveData.buf[7], 2))
        {
          debugPrintln("\r\nHardware protection");
        }
        else if (bitRead(KeyaBusReceiveData.buf[7], 3))
        {
          debugPrintln("\r\nE2PROM");
        }
        else if (bitRead(KeyaBusReceiveData.buf[7], 4))
        {
          debugPrintln("\r\nUnder voltage");
        }
        else if (bitRead(KeyaBusReceiveData.buf[7], 5))
        {
          debugPrintln("\r\nN/A");
        }
        else if (bitRead(KeyaBusReceiveData.buf[7], 6))
        {
          debugPrintln("\r\nOver current");
        }
        else if (bitRead(KeyaBusReceiveData.buf[7], 7))
        {
          debugPrintln("\r\nMode failure");
        }
      }

      if (KeyaBusReceiveData.buf[6] != 0)
      {
        if (bitRead(KeyaBusReceiveData.buf[6], 0))
        {
          debugPrint("\r\nLess phase");
        }
        else if (bitRead(KeyaBusReceiveData.buf[6], 1))
        {
          debugPrintln("\r\nMotor stall");
        }
        else if (bitRead(KeyaBusReceiveData.buf[6], 2))
        {
          debugPrintln("\r\nReserved");
        }
        else if (bitRead(KeyaBusReceiveData.buf[6], 3))
        {
          debugPrintln("\r\nHall failure");
        }
        else if (bitRead(KeyaBusReceiveData.buf[6], 4))
        {
          debugPrintln("\r\nCurrent sensing");
        }
        else if (bitRead(KeyaBusReceiveData.buf[6], 5))
        {
          debugPrintln("\r\n232 disconnected");
        }
        else if (bitRead(KeyaBusReceiveData.buf[6], 6))
        {
          if(debugState == KEYA)
            debugPrintln("\r\nCAN disconnected");
        }
        else if (bitRead(KeyaBusReceiveData.buf[6], 7))
        {
          debugPrintln("\r\nMotor stalled");
        }
      }
      // keyaCommand(keyaTemperatureQuery);
      // keyaCommand(keyaVoltageQuery);
      // keyaCommand(keyaFaultQuery);
    }

    // parse query/command 00:05:08:00:00:00:00:[ID] responses
    if (KeyaBusReceiveData.id == 0x05800001)
    {

      // Disable command response
      if (isPatternMatch(KeyaBusReceiveData, keyaDisableResponse, sizeof(keyaDisableResponse)))
      {
        // printIdAndReply(KeyaBusReceiveData.id, KeyaBusReceiveData.buf);
        // debugPrint(" disable reply ");
      }

      // Enable command response
      else if (isPatternMatch(KeyaBusReceiveData, keyaEnableResponse, sizeof(keyaEnableResponse)))
      {
        // printIdAndReply(KeyaBusReceiveData.id, KeyaBusReceiveData.buf);
        // debugPrint(" enable reply ");
      }

      // Speed command response
      else if (isPatternMatch(KeyaBusReceiveData, keyaSpeedResponse, sizeof(keyaSpeedResponse)))
      {
        // printIdAndReply(KeyaBusReceiveData.id, KeyaBusReceiveData.buf);
        // debugPrint(" speed reply ");
      }

      // Current query response (this is also in heartbeat)
      else if (isPatternMatch(KeyaBusReceiveData, keyaCurrentResponse, sizeof(keyaCurrentResponse)))
      {
        uint32_t time = millis();
        keyaTime = time;
        printIdAndReply(KeyaBusReceiveData.id, KeyaBusReceiveData.buf);
        if(debugState == KEYA){
          debugPrint(" current reply ");
          debugPrint(KeyaBusReceiveData.buf[4]);
        }
        KeyaCurrentSensorReading = KeyaBusReceiveData.buf[4];
      }

      // Encoder query response
      else if (isPatternMatch(KeyaBusReceiveData, keyaEncoderResponse, sizeof(keyaEncoderResponse)))
      {
        printIdAndReply(KeyaBusReceiveData.id, KeyaBusReceiveData.buf);
        keyaEncoderValue = KeyaBusReceiveData.buf[7] << 24 | 
            KeyaBusReceiveData.buf[6] << 16 | 
            KeyaBusReceiveData.buf[5] << 8 | 
            KeyaBusReceiveData.buf[4];
        if(debugState == KEYA){
          debugPrint(" encoder reply ");
          debugPrint(keyaEncoderValue);
        }

        //so right is positive
        keyaEncoderValue=keyaEncoderValue*-1;

        if(keyaEncoderValueOld>keyaEncoderValue)
          keyaDir=-1;
        else if(keyaEncoderValueOld<keyaEncoderValue)
          keyaDir=1;
        keyaEncoderValueOld = keyaEncoderValue;


        switch (keyaState)
        {
        case 0:       //start point
          if(keyaDir==1)
            keyaState=1;
          else
            keyaState=3;
          break;
        
        case 1:     //giro a dx
          if(keyaDir==-1)
            keyaState=2;
          else{
            keyaEncoderValueFreeze=keyaEncoderValue;
          }
          break;
        
        case 2:     //cambio verso sx
          if(keyaEncoderValueFreeze-keyaEncoderValue>steerSettings.keyaDirOffset)
            keyaState=3;
          else if(keyaEncoderValue>keyaEncoderValueFreeze)
            keyaState=1;
          keyaEncoderValue=keyaEncoderValueFreeze;
          break;
        
        case 3:     //giro a sx
          keyaEncoderValue += steerSettings.keyaDirOffset;
          if(keyaDir==1)
            keyaState=4;
          else{
            keyaEncoderValueFreeze=keyaEncoderValue;
          }
          break;
        
        case 4:     //cambio a dx
          keyaEncoderValue += steerSettings.keyaDirOffset;
          if(keyaEncoderValue-keyaEncoderValueFreeze>steerSettings.keyaDirOffset)
            keyaState=1;
          else if(keyaEncoderValue<keyaEncoderValueFreeze)
            keyaState=3;

          keyaEncoderValue=keyaEncoderValueFreeze;
          break;
        
        default:
          keyaState=0;
          break;
        }

        if(steerSettings.keyaAckermanFix != 100){
          keyaEncoderDiff = keyaEncoderValue - keyaEncoderFinalOld;
          
          if(keyaDir==1) // turning right
            keyaEncoderDiff *= steerSettings.keyaAckermanFix;
          else
            keyaEncoderDiff *= 100;
          
          keyaEncoderVirtual += keyaEncoderDiff;
          keyaEncoder = (float)keyaEncoderVirtual / 600.0f;
          keyaEncoderFinalOld = keyaEncoderValue;
        }
        else
          keyaEncoder = (float)keyaEncoderValue / 6.0f;
      }

      // Encoder speed query response
      else if (isPatternMatch(KeyaBusReceiveData, keyaEncoderSpeedResponse, sizeof(keyaEncoderSpeedResponse)))
      {
        printIdAndReply(KeyaBusReceiveData.id, KeyaBusReceiveData.buf);
        keyaEncoderSpeed = KeyaBusReceiveData.buf[5] << 8 | KeyaBusReceiveData.buf[4];
        if(keyaEncoderSpeed>65000)
          keyaEncoderSpeed=keyaEncoderSpeed-65536;
        if(debugState == KEYA){
          debugPrint(" encoder speed reply ");
          debugPrint(keyaEncoderSpeed);
        }

        if(keyaEncoderSpeed!=0){
          KeyaCurrentRapport = KeyaCurrentSensorReading/abs(keyaEncoderSpeed)*200;
          KeyaCurrentRapportSmooth = KeyaCurrentRapportSmooth*0.7 + KeyaCurrentRapport*0.3;

          if(debugState == KEYA){
          debugPrint(" current/speed ");
          debugPrint(KeyaCurrentRapportSmooth);
          }
        }
      }

      // Fault query response
      else if (isPatternMatch(KeyaBusReceiveData, keyaFaultResponse, sizeof(keyaFaultResponse)))
      {
        printIdAndReply(KeyaBusReceiveData.id, KeyaBusReceiveData.buf);
        debugPrint(" fault reply ");
        debugPrint(KeyaBusReceiveData.buf[4]);
        debugPrint(":");
        debugPrint(KeyaBusReceiveData.buf[5]);
      }

      // Voltage query response
      else if (isPatternMatch(KeyaBusReceiveData, keyaVoltageResponse, sizeof(keyaVoltageResponse)))
      {
        printIdAndReply(KeyaBusReceiveData.id, KeyaBusReceiveData.buf);
        debugPrint(" voltage reply ");
        debugPrint(KeyaBusReceiveData.buf[4]);
      }

      // Temperature query response
      else if (isPatternMatch(KeyaBusReceiveData, keyaTemperatureResponse, sizeof(keyaTemperatureResponse)))
      {
        printIdAndReply(KeyaBusReceiveData.id, KeyaBusReceiveData.buf);
        debugPrint(" temperature reply ");
        debugPrint(KeyaBusReceiveData.buf[4]);
      }

      // Version query response
      else if (isPatternMatch(KeyaBusReceiveData, keyaVersionResponse, sizeof(keyaVersionResponse)))
      {
        printIdAndReply(KeyaBusReceiveData.id, KeyaBusReceiveData.buf);
        debugPrint(" version reply ");
        debugPrint(KeyaBusReceiveData.buf[4]);
        debugPrint(":");
        debugPrint(KeyaBusReceiveData.buf[5]);
        debugPrint(":");
        debugPrint(KeyaBusReceiveData.buf[6]);
        debugPrint(":");
        debugPrint(KeyaBusReceiveData.buf[7]);
      }
      else
      {
        printIdAndReply(KeyaBusReceiveData.id, KeyaBusReceiveData.buf);
        debugPrint(" unknown reply ");
      }
    }

    if (lnNeeded)
    {
      debugPrintln();
      lnNeeded = false;
    }
  }
}

void getKeyaInfo(){
  keyaCommand(keyaVoltageQuery);
  delay(10);
  KeyaBus_Receive();
  delay(10);

  keyaCommand(keyaTemperatureQuery);
  delay(10);
  KeyaBus_Receive();
  delay(10);

  keyaCommand(keyaVersionQuery);
  delay(10);
  KeyaBus_Receive();
}