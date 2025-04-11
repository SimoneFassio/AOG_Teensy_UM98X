void EthernetStart()
{
#ifdef ARDUINO_TEENSY41
  // start the Ethernet connection:
  debugPrintln("Initializing ethernet with static IP address");

  // try to congifure using IP:
  Ethernet.begin(mac, 0); // Start Ethernet with IP 0.0.0.0

  // Check for Ethernet hardware present
  if (Ethernet.hardwareStatus() == EthernetNoHardware)
  {
    debugPrintln("Ethernet shield was not found. GPS via USB only.");

    return;
  }

  if (Ethernet.linkStatus() == LinkOFF)
  {
    debugPrintln("Ethernet cable is not connected - Who cares we will start ethernet anyway.");
  }

  // grab the ip from EEPROM
  Eth_myip[0] = networkAddress.ipOne;
  Eth_myip[1] = networkAddress.ipTwo;
  Eth_myip[2] = networkAddress.ipThree;

  if (Autosteer_running)
  {
    Eth_myip[3] = 126; // 126 is steer module, with or without GPS
  }
  else
  {
    Eth_myip[3] = 120; // 120 is GPS only module
  }

  Ethernet.setLocalIP(Eth_myip); // Change IP address to IP set by user
  debugPrintln("\r\nEthernet status OK");
  debugPrint("IP set Manually: ");
  debugPrintln(Ethernet.localIP());

  Eth_ipDestination[0] = Eth_myip[0];
  Eth_ipDestination[1] = Eth_myip[1];
  Eth_ipDestination[2] = Eth_myip[2];
  Eth_ipDestination[3] = 255;

  debugPrint("\r\nEthernet IP of module: ");
  debugPrintln(Ethernet.localIP());
  debugPrint("Ethernet sending to IP: ");
  debugPrintln(Eth_ipDestination);
  debugPrint("All data sending to port: ");
  debugPrintln(portDestination);

  // init UPD Port sending to AOG
  if (Eth_udpPAOGI.begin(portMy))
  {
    debugPrint("Ethernet GPS UDP sending from port: ");
    debugPrintln(portMy);
  }

  // init UPD Port getting NTRIP from AOG
  if (Eth_udpNtrip.begin(AOGNtripPort)) // AOGNtripPort
  {
    debugPrint("Ethernet NTRIP UDP listening to port: ");
    debugPrintln(AOGNtripPort);
  }

  // init UPD Port getting AutoSteer data from AOG
  if (Eth_udpAutoSteer.begin(AOGAutoSteerPort)) // AOGAutoSteerPortipPort
  {
    debugPrint("Ethernet AutoSteer UDP listening to & send from port: ");
    debugPrintln(AOGAutoSteerPort);
  }

  // init UDP debug IN port
  if (Eth_udpDebug.begin(portDebugIN))
  {
    debugPrint("Ethernet Debug UDP listening from port: ");
    debugPrintln(portDebugIN);
  }

  Ethernet_running = true;
#endif
}
