// Conversion to Hexidecimal
const char *asciiHex = "0123456789ABCDEF";

SimpleKalmanFilter rollKF(0.1, 0.1, 0.1);

// the new PANDA sentence buffer

char nmea[110];

// GGA
char fixTime[12];
char latitude[15];
char latNS[3];
char longitude[15];
char lonEW[3];
char fixQuality[2];
char numSats[4];
char HDOP[5];
char altitude[12];
char ageDGPS[10];

// VTG
char vtgHeading[12] = {};
char speedKnots[10] = {};

// INS
bool insActive=false;
bool insStart=false;
uint8_t index1=0;
uint8_t numArg;
char bufferSERIAL[40];
char insStatus[25];
char insLatitude[16];
char insLongitude[16];
//char velocity[16];

// HPR
char solQuality[2];


// If odd characters showed up.
void errorHandler()
{
  //if(debugState == GPS || send_GPS)
    //debugPrintln("parser error");
    return;
}

void GGA_Handler() // Rec'd GGA
{
  // fix time
  parser.getArg(0, fixTime);

  // latitude
  if(!usingUM982 && !UM981_aligned) //else use the one from INS
    parser.getArg(1, latitude);
  parser.getArg(2, latNS);

  // longitude
  if(!usingUM982 && !UM981_aligned)
    parser.getArg(3, longitude);
  parser.getArg(4, lonEW);

  // fix quality
  parser.getArg(5, fixQuality);

  // satellite #
  parser.getArg(6, numSats);

  // HDOP
  parser.getArg(7, HDOP);

  // altitude
  parser.getArg(8, altitude);

  // time of last DGPS update
  parser.getArg(12, ageDGPS);

  if (strstr(insStatus, "INS_SOLUTION")!=NULL) // led always on if INS is aligned
  {
    digitalWrite(GGAReceivedLED, HIGH);
  }
  else if (blink)
  {
    digitalWrite(GGAReceivedLED, HIGH);
  }
  else
  {
    digitalWrite(GGAReceivedLED, LOW);
  }

  blink = !blink;

  gpsReadyTime = systick_millis_count; // Used for GGA timeout (LED's ETC)

  if(debugState == GPS || send_GPS){
    debugPrint(systick_millis_count);
    debugPrint("  GGA  ");
  }
}

void VTG_Handler()
{
  // vtg heading
  parser.getArg(0, vtgHeading);  //i can use for reverse detection (180° from dualHeading)

  // vtg Speed knots
  parser.getArg(4, speedKnots);
  if(atof(speedKnots)<0.05)
    snprintf(speedKnots, 5, "0.0");
  speed = atof(speedKnots) * 1852 / 3600; // m/s

  // if(debugState == EXPERIMENT){
  //   debugPrint(" VTGspeed:");
  //   debugPrint(speed);
  //   debugPrint("  ");
  // }

  float headingVTG = atof(vtgHeading);
  if(abs((int)(headingVTG-atof(insHeading))%360)>120 && speed>0.5)  //reverse
    workingDir=-1;
  else
    workingDir=1;

  if(debugState == GPS || send_GPS){
    debugPrint(systick_millis_count);
    debugPrint("  VTG  ");
  }
}

void DegreesToDegMinSec(double x, char* result, int resultSize)
{
  int deg = (int)floor(x);
  double minutes = fabs(x - deg) * 60.0;

  if (minutes < 10.0)
    snprintf(result, resultSize, "%02d0%.8f", deg, minutes);
  else
    snprintf(result, resultSize, "%02d%.8f",  deg, minutes);
}

void readSerialIns(char c){
      if(c == '#' || insActive){
        insActive = true;
        if (c == ';'){
          numArg = 2;
          index1=0;
          insStart=true;
          for(int g=0; g<40; g++)
            bufferSERIAL[g] = '\0';
          if(debugState == GPS || send_GPS){
            debugPrint(systick_millis_count);
            debugPrint("  INS  ");
          }
        }
        else if(c == ',' && insStart){
          index1++;
          switch(numArg){
            case 2: //INSstatus
              strncpy(insStatus, bufferSERIAL, sizeof(insStatus));
              insStatus[24] = '\0';
              if(debugState == GPS || send_GPS){
                debugPrint("INSstatus: ");
                debugPrint(bufferSERIAL);
                debugPrint("  ");
              }
              break;
            case 4: { //latitude
              if(debugState == GPS || send_GPS){
                debugPrint("latitude: ");
                debugPrint(bufferSERIAL);
                debugPrint("  ");
              }
              bufferSERIAL[15] = '\0';
              double lat = strtod(bufferSERIAL, NULL);
              if (lat == 0.0) {
                strncpy(insLatitude, latitude, sizeof(insLatitude));
                UM981_aligned = false;
              } else {
                DegreesToDegMinSec(lat, insLatitude, sizeof(insLatitude));
                UM981_aligned = true;
              }
              break;
            }

            case 5: { //longitude
              if(debugState == GPS || send_GPS){
                debugPrint("longitude: ");
                debugPrint(bufferSERIAL);
                debugPrint("  ");
              }
              bufferSERIAL[15] = '\0';
              double lon = strtod(bufferSERIAL, NULL);
              if (lon == 0.0) {
                strncpy(insLongitude, longitude, sizeof(insLongitude));
              } else {
                DegreesToDegMinSec(lon, insLongitude, sizeof(insLongitude));
              }
              break;
            }

            case 11: //roll
              if(debugState == GPS || send_GPS){
                debugPrint("roll:");
                debugPrint(bufferSERIAL);
                debugPrint("  ");
              }
              for(int g=0; g<5; g++)
                insRoll[g] = bufferSERIAL[g];
              insRoll[5] = '\0';
              break;
            case 12: //pitch
              if(debugState == GPS || send_GPS){
                debugPrint("pitch: ");
                debugPrint(bufferSERIAL);
                debugPrint("  ");
              }
              for(int g=0; g<5; g++)
                insPitch[g] = bufferSERIAL[g];
              insPitch[5] = '\0';
              break;
            case 13: //heading
              if(debugState == GPS || send_GPS){
                debugPrint("heading: ");
                debugPrint(bufferSERIAL);
                debugPrint("  ");
              }
              for(int g=0; g<6; g++)
                insHeading[g] = bufferSERIAL[g];
              insHeading[6] = '\0';
              break;
            // case 8: //north vel
            //   if(debugState == EXPERIMENT){
            //     debugPrint("north_vel: ");
            //     debugPrint(atof(bufferSERIAL));
            //   }
            //   for(int g=0; g<6; g++)
            //     velocity[g] = bufferSERIAL[g];
            //   velocity[6] = '\0';
            //   break;
            // case 9: //east vel
            //   if(debugState == EXPERIMENT){
            //     debugPrint(" east_vel: ");
            //     debugPrint(atof(bufferSERIAL));
            //   }
            //   insSpeed = sqrt(sq(atof(velocity))+sq(atof(bufferSERIAL)));
            //   if(debugState == GPS || debugState == EXPERIMENT){
            //     debugPrint(" vel: ");
            //     debugPrintln(insSpeed);
            //   }
            //   break;
          }
          numArg++;
          for(int g=0; g<20; g++)
            bufferSERIAL[g] = '\0';
          index1=0;
        }
        else if(c == '*' && insStart){
          insActive = false;
          insStart = false;
          dualReadyINS++;
          angleStimeUpdate();
        }
        else if(insStart) {
          bufferSERIAL[index1] = c;
          index1++;
        }
      }
      else if(!settings.using2serialGPS)
        parser << c;
}

// UM982 Support
void HPR_Handler()
{
  // make sure we are using UM982, even if not got during setup
  usingUM982 = true;

  // HPR Heading
  parser.getArg(1, umHeading);
  umHeading[6] = '\0';

  // Solution quality factor
  // parser.getArg(4, solQuality);

  // HPR Substitute pitch for roll
  if (parser.getArg(2, umRoll))
  {
    rollDual = atof(umRoll);

    if(debugState == ROLL){
      debugPrint("rollDual:");
      debugPrint(rollDual);
    }

    rollDual = rollKF.updateEstimate(rollDual);

    if(debugState == ROLL){
      debugPrint("rollKF:");
      debugPrint(rollDual);
      debugPrint(",angleWT:");
      debugPrintln(rollWT, 3);
    }
  }
  
  dualReadyINS++;

  if(debugState == GPS || send_GPS)
    debugPrintln("  HPR  ");

  angleStimeUpdate();
}

void BuildNmea(void)
{
  strcpy(nmea, "");

  if (makeOGI)
    strcat(nmea, "$PAOGI,");
  else
    strcat(nmea, "$PANDA,");

  strcat(nmea, fixTime);
  strcat(nmea, ",");

  if(!usingUM982) {
    strcat(nmea, insLatitude);
    strcat(nmea, ",");

    strcat(nmea, latNS);
    strcat(nmea, ",");

    strcat(nmea, insLongitude);
    strcat(nmea, ",");

    strcat(nmea, lonEW);
    strcat(nmea, ",");
  }
  else {
    strcat(nmea, latitude);
    strcat(nmea, ",");

    strcat(nmea, latNS);
    strcat(nmea, ",");

    strcat(nmea, longitude);
    strcat(nmea, ",");

    strcat(nmea, lonEW);
    strcat(nmea, ",");
  }

  // 6
  strcat(nmea, fixQuality);
  strcat(nmea, ",");

  strcat(nmea, numSats);
  strcat(nmea, ",");

  strcat(nmea, HDOP);
  strcat(nmea, ",");

  strcat(nmea, altitude);
  strcat(nmea, ",");

  // 10
  strcat(nmea, ageDGPS);
  strcat(nmea, ",");

  // 11
  strcat(nmea, speedKnots);
  strcat(nmea, ",");

  // 12
  strcat(nmea, headingPanda);
  strcat(nmea, ",");

  // 13
  strcat(nmea, rollPanda);
  strcat(nmea, ",");

  // 14
  strcat(nmea, pitchPanda);
  strcat(nmea, ",");

  // 15
  strcat(nmea, yawRatePanda);

  strcat(nmea, "*");

  CalculateChecksum();

  strcat(nmea, "\r\n");

  if (sendUSB)
  {
    Serial.write(nmea);
  } // Send USB GPS data if enabled in user settings

  if (Ethernet_running) // If ethernet running send the GPS there
  {
    int len = strlen(nmea);
    Eth_udpPAOGI.beginPacket(Eth_ipDestination, portDestination);
    Eth_udpPAOGI.write(nmea, len);
    Eth_udpPAOGI.endPacket();
  }

  if(debugState == GPS || send_GPS)
    debugPrintln("NMEA Sent");
}

void CalculateChecksum(void)
{
  int16_t sum = 0;
  int16_t inx = 0;
  char tmp;

  // The checksum calc starts after '$' and ends before '*'
  for (inx = 1; inx < 200; inx++)
  {
    tmp = nmea[inx];

    // * Indicates end of data and start of checksum
    if (tmp == '*')
    {
      break;
    }

    sum ^= tmp; // Build checksum
  }

  byte chk = (sum >> 4);
  char hex[2] = {asciiHex[chk], 0};
  strcat(nmea, hex);

  chk = (sum % 16);
  char hex2[2] = {asciiHex[chk], 0};
  strcat(nmea, hex2);
}

/*
  $PANDA
  (1) Time of fix

  position
  (2,3) 4807.038,N Latitude 48 deg 07.038' N
  (4,5) 01131.000,E Longitude 11 deg 31.000' E

  (6) 1 Fix quality:
    0 = invalid
    1 = GPS fix(SPS)
    2 = DGPS fix
    3 = PPS fix
    4 = Real Time Kinematic
    5 = Float RTK
    6 = estimated(dead reckoning)(2.3 feature)
    7 = Manual input mode
    8 = Simulation mode
  (7) Number of satellites being tracked
  (8) 0.9 Horizontal dilution of position
  (9) 545.4 Altitude (ALWAYS in Meters, above mean sea level)
  (10) 1.2 time in seconds since last DGPS update
  (11) Speed in knots

  FROM IMU:
  (12) Heading in degrees
  (13) Roll angle in degrees(positive roll = right leaning - right down, left up)

  (14) Pitch angle in degrees(Positive pitch = nose up)
  (15) Yaw Rate in Degrees / second

  CHKSUM
*/

/*
  $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M ,  ,*47
   0     1      2      3    4      5 6  7  8   9    10 11  12 13  14
        Time      Lat       Lon     FixSatsOP Alt
  Where:
     GGA          Global Positioning System Fix Data
     123519       Fix taken at 12:35:19 UTC
     4807.038,N   Latitude 48 deg 07.038' N
     01131.000,E  Longitude 11 deg 31.000' E
     1            Fix quality: 0 = invalid
                               1 = GPS fix (SPS)
                               2 = DGPS fix
                               3 = PPS fix
                               4 = Real Time Kinematic
                               5 = Float RTK
                               6 = estimated (dead reckoning) (2.3 feature)
                               7 = Manual input mode
                               8 = Simulation mode
     08           Number of satellites being tracked
     0.9          Horizontal dilution of position
     545.4,M      Altitude, Meters, above mean sea level
     46.9,M       Height of geoid (mean sea level) above WGS84
                      ellipsoid
     (empty field) time in seconds since last DGPS update
     (empty field) DGPS station ID number
      47          the checksum data, always begins with


  $GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A
  0      1    2   3      4    5      6   7     8     9     10   11
        Time      Lat        Lon       knots  Ang   Date  MagV

  Where:
     RMC          Recommended Minimum sentence C
     123519       Fix taken at 12:35:19 UTC
     A            Status A=active or V=Void.
     4807.038,N   Latitude 48 deg 07.038' N
     01131.000,E  Longitude 11 deg 31.000' E
     022.4        Speed over the ground in knots
     084.4        Track angle in degrees True
     230394       Date - 23rd of March 1994
     003.1,W      Magnetic Variation
      6A          The checksum data, always begins with

  $GPVTG,054.7,T,034.4,M,005.5,N,010.2,K*48

    VTG          Track made good and ground speed
    054.7,T      True track made good (degrees)
    034.4,M      Magnetic track made good
    005.5,N      Ground speed, knots
    010.2,K      Ground speed, Kilometers per hour
     48          Checksum
*/