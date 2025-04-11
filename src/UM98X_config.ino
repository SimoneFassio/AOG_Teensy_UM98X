
void checkUM981(){
  uint32_t baudrate = 0;
  for (uint32_t i = 0; i < nrBaudrates; i++)
  {
    baudrate = baudrates[i];
    debugPrint(F("Checking for UM981 at baudrate: "));
    debugPrintln(baudrate);
    SerialGPS.begin(baudrate);
    // Increase the size of the serial buffer to hold longer UM981 config messages
    SerialGPS.addMemoryForRead(tmpGPSrxbuffer, tmp_serial_buffer_size);
    SerialGPS.addMemoryForWrite(tmpGPStxbuffer, tmp_serial_buffer_size);
    delay(100);
    SerialGPS.write("VERSION\r\n");
    delay(100);
    while (SerialGPS.available())
    {
      char incoming[300];
      SerialGPS.readBytesUntil('\n', incoming, 300);
      if (strstr(incoming, "UM981") != NULL)
      {
        debugPrint("UM981 VERSION: ");
        debugPrintln(incoming);
        if (baudrate != 460800)
        {
          debugPrintln("UM981 baudrate wrong for AOG. Setting to 460800 bps for AOG");
          SerialGPS.write("CONFIG COM1 460800\r\n");
          delay(100);
          SerialGPS.begin(baudGPS);
        }
        gotUM981 = true;
        break;
      }
      if (gotUM981)
      {
        break;
      }
    }
    if (gotUM981)
    {
      break;
    }
  }
}

void configureUM981(){
  SerialGPS.write("CONFIG\r\n"); // Request the UM981 Configuration
  delay(200);

  while (SerialGPS.available() && !setUM981)
  {
    char incoming[300];
    SerialGPS.readBytesUntil('\n',incoming, 300);
    setUM981=false;

    // Check the "UM981 configured" flag.
    if (strstr(incoming, "CONFIG ANTENNADELTAHEN") != NULL)
    {
      debugPrintln("Got the config line");
      char buffer[100];
      sprintf(buffer, "CONFIG ANTENNADELTAHEN %.2f", calibrationData.configFlag);
      if (strstr(incoming, buffer) != NULL)
      {
        debugPrintln("And it is already configured");
        debugPrintln();

        // Reset serial buffer size
        SerialGPS.addMemoryForRead(GPSrxbuffer, serial_buffer_size);
        SerialGPS.addMemoryForWrite(GPStxbuffer, serial_buffer_size);
        setUM981 = true;
      }
      else
      {
        debugPrintln("And it is not already configured");

        // Clear out the serial channel
        SerialGPS.write("UNLOGALL\r\n");
        SerialGPS.write("UNLOGALL COM2\r\n");
        while ((SerialGPS.available()))
        {
          SerialGPS.read();
        }
        // Set UM981 operating mode
        debugPrintln("Setting rover mode");
        SerialGPS.write("MODE ROVER SURVEY\r\n");
        delay(100);

        // Send IMUTOANT configuration
        debugPrintln("Setting INS");
        SerialGPS.print("CONFIG IMUTOANT OFFSET ");
        SerialGPS.print(calibrationData.IMUtoANTx, 2);
        SerialGPS.print(" ");
        SerialGPS.print(calibrationData.IMUtoANTy, 2);
        SerialGPS.print(" ");
        SerialGPS.print(calibrationData.IMUtoANTz, 2);
        SerialGPS.println(" 0.01 0.01 0.01\r\n");
        delay(100);

        // Send INS ANGLE configuration
        SerialGPS.print("CONFIG INSSOL OFFSET ");
        SerialGPS.print(calibrationData.INSx, 2);
        SerialGPS.print(" ");
        SerialGPS.print(calibrationData.INSy, 2);
        SerialGPS.print(" ");
        SerialGPS.print(calibrationData.INSz, 2);
        SerialGPS.println("\r\n");
        delay(100);

        // Send INSSOL OFFSET configuration
        SerialGPS.print("CONFIG INS ANGLE ");
        SerialGPS.print((int)(calibrationData.INSanglex * 100));
        SerialGPS.print(" ");
        SerialGPS.print((int)(calibrationData.INSangley * 100));
        SerialGPS.print(" ");
        SerialGPS.print((int)(calibrationData.INSanglez * 100));
        SerialGPS.println("\r\n");
        delay(100);

        SerialGPS.write("CONFIG INS ALIGNMENTVEL 1.2\r\n");  // 1 m/s
        delay(100);

        // debugPrintln("Setting PPP");
        // SerialGPS.write("CONFIG PPP ENABLE AUTO\r\n");  //E6-HAS
        // delay(100);

        // SerialGPS.write("CONFIG PPP DATUM WGS84\r\n");
        // delay(100);

        // SerialGPS.write("CONFIG PPP CONVERGE 50 50\r\n");
        // delay(100);

        // SerialGPS.write("CONFIG SIGNALGROUP 8\r\n");
        // delay(100);

        // Set rtk height smoothing
        debugPrintln("Setting rtkheight smoothing");
        SerialGPS.write("CONFIG SMOOTH RTKHEIGHT 10\r\n");
        delay(100);

        // Set COM1 to 460800
        debugPrintln("Setting COM1 to 460800 bps");
        SerialGPS.write("CONFIG COM1 460800\r\n");
        delay(100);

        // Set COM2 to 460800
        debugPrintln("Setting COM2 to 460800 bps");
        SerialGPS.write("CONFIG COM2 460800\r\n");
        delay(100);

        // Set GGA message and rate
        debugPrintln("Setting GGA");
        SerialGPS.write("GNGGA COM1 0.1\r\n");
        delay(100);

        // Set VTG message and rate
        debugPrintln("Setting VTG");
        SerialGPS.write("GPVTG COM1 0.1\r\n");
        delay(100);

        // Set INS message and rate
        debugPrintln("Setting INS");
        SerialGPS.write("INSPVAXA ");
        if (settings.using2serialGPS)
          SerialGPS.write("COM2 ");
        else
          SerialGPS.write("COM1 ");
        SerialGPS.print(settings.intervalINS, 2);
        SerialGPS.write("\r\n");
        delay(100);

        // Setting the flag to signal UM981 is configured for AOG
        debugPrintln("Setting UM981 configured flag");
        SerialGPS.write("CONFIG ANTENNADELTAHEN ");
        SerialGPS.print(calibrationData.configFlag, 2);
        SerialGPS.write("\r\n");
        delay(100);

        // Saving the configuration in the UM981
        debugPrintln("Saving the configuration");
        SerialGPS.write("SAVECONFIG\r\n");
        debugPrintln();
        delay(100);

        while (SerialGPS.available())
        {
          SerialGPS.readBytesUntil('\n', incoming, 300);
          if (strncmp(incoming, "$command", 8) == 0) // Check if the message starts with "$command"
          {
            debugPrintln(incoming); // Print the message if it matches
          }
        }

        // Reset the serial buffer size
        SerialGPS.addMemoryForRead(GPSrxbuffer, serial_buffer_size);
        SerialGPS.addMemoryForWrite(GPStxbuffer, serial_buffer_size);
      }
    }
  }
}


void checkUM982(){
  uint32_t baudrate = 0;
  for (uint32_t i = 0; i < nrBaudrates; i++)
  {
    baudrate = baudrates[i];
    debugPrint(F("Checking for UM982 at baudrate: "));
    debugPrintln(baudrate);
    SerialGPS.begin(baudrate);
    // Increase the size of the serial buffer to hold longer UM982 config messages
    SerialGPS.addMemoryForRead(tmpGPSrxbuffer, tmp_serial_buffer_size);
    SerialGPS.addMemoryForWrite(tmpGPStxbuffer, tmp_serial_buffer_size);
    delay(100);
    SerialGPS.write("VERSION\r\n");
    delay(100);
    while (SerialGPS.available())
    {
      char incoming[300];
      SerialGPS.readBytesUntil('\n', incoming, 300);
      if (strstr(incoming, "UM982") != NULL)
      {
        debugPrint("UM982 VERSION: ");
        debugPrintln(incoming);
        if (baudrate != 460800)
        {
          debugPrintln("UM982 baudrate wrong for AOG. Setting to 460800 bps for AOG");
          SerialGPS.write("CONFIG COM1 460800\r\n");
          delay(100);
          SerialGPS.begin(baudGPS);
        }
        gotUM982 = true;
        break;
      }
      if (gotUM982)
      {
        break;
      }
    }
    if (gotUM982)
    {
      break;
    }
  }
}

void configureUM982(){
  SerialGPS.write("CONFIG\r\n"); // Request the UM982 Configuration
  delay(200);

  while (SerialGPS.available() && !setUM982)
  {
    char incoming[300];
    SerialGPS.readBytesUntil('\n',incoming, 300);
    setUM982=false;

    // Check the "UM982 configured" flag.
    if (strstr(incoming, "CONFIG ANTENNADELTAHEN") != NULL)
    {
      debugPrintln("Got the config line");
      char buffer[100];
      sprintf(buffer, "CONFIG ANTENNADELTAHEN %.2f", calibrationData.configFlag);
      if (strstr(incoming, buffer) != NULL)
      {
        debugPrintln("And it is already configured");
        debugPrintln();

        // Reset serial buffer size
        SerialGPS.addMemoryForRead(GPSrxbuffer, serial_buffer_size);
        SerialGPS.addMemoryForWrite(GPStxbuffer, serial_buffer_size);
        setUM982 = true;
      }
      else
      {
        debugPrintln("And it is not already configured");

        // Clear out the serial channel
        SerialGPS.write("UNLOGALL\r\n");
        SerialGPS.write("UNLOGALL COM2\r\n");
        while ((SerialGPS.available()))
        {
          SerialGPS.read();
        }
        // Set UM982 operating mode
        debugPrintln("Setting rover mode");
        SerialGPS.write("MODE ROVER SURVEY\r\n");
        delay(100);

        SerialGPS.write("CONFIG PPP ENABLE AUTO\r\n");  //E6-HAS PARSING FAILD GRAMMAR ERROR
        delay(100);

        SerialGPS.write("CONFIG PPP DATUM WGS84\r\n");
        delay(100);

        SerialGPS.write("CONFIG PPP CONVERGE 50 50\r\n");
        delay(100);

        SerialGPS.write("CONFIG SIGNALGROUP 8\r\n");    // PARSING FAILD GRAMMAR ERROR
        delay(100);

        // Set rtk height smoothing
        debugPrintln("Setting rtkheight smoothing");
        SerialGPS.write("CONFIG SMOOTH RTKHEIGHT 10\r\n");
        delay(100);

        // Set heading offset
        debugPrintln("Setting heading offset 270");
        SerialGPS.write("CONFIG HEADING OFFSET -90\r\n"); //dx is main antenna
        delay(100);

        // Set heading smoothing
        debugPrintln("Setting heading smoothing");
        SerialGPS.write("CONFIG SMOOTH HEADING 1\r\n");
        delay(100);

        // Set COM1 to 460800
        debugPrintln("Setting COM1 to 460800 bps");
        SerialGPS.write("CONFIG COM1 460800\r\n");
        delay(100);

        // Set COM2 to 460800
        debugPrintln("Setting COM2 to 460800 bps");
        SerialGPS.write("CONFIG COM2 460800\r\n");
        delay(100);

        // Set GGA message and rate
        debugPrintln("Setting GGA");
        SerialGPS.write("GNGGA COM1 0.1\r\n");
        delay(100);

        // Set VTG message and rate
        debugPrintln("Setting VTG");
        SerialGPS.write("GPVTG COM1 0.1\r\n");
        delay(100);

        // Set INS message and rate
        debugPrintln("Setting HPR");
        SerialGPS.write("GPHPR ");
        if (settings.using2serialGPS)
          SerialGPS.write("COM2 ");
        else
          SerialGPS.write("COM1 ");
        SerialGPS.print(settings.intervalINS, 2);
        SerialGPS.write("\r\n");
        delay(100);

        // Setting the flag to signal UM982 is configured for AOG
        debugPrintln("Setting UM982 configured flag");
        SerialGPS.write("CONFIG ANTENNADELTAHEN ");
        SerialGPS.print(calibrationData.configFlag, 2);
        SerialGPS.write("\r\n");
        delay(100);

        // Saving the configuration in the UM982
        debugPrintln("Saving the configuration");
        SerialGPS.write("SAVECONFIG\r\n");
        debugPrintln();
        delay(100);

        while (SerialGPS.available())
        {
          SerialGPS.readBytesUntil('\n', incoming, 300);
          if (strncmp(incoming, "$command", 8) == 0) // Check if the message starts with "$command"
          {
            debugPrintln(incoming); // Print the message if it matches
          }
        }

        // Reset the serial buffer size
        SerialGPS.addMemoryForRead(GPSrxbuffer, serial_buffer_size);
        SerialGPS.addMemoryForWrite(GPStxbuffer, serial_buffer_size);
      }
    }
  }
}
