// Copyright - Jared Gaillard - 2016
// MIT License
//
// Arduino CZII Project
//
//   https://github.com/jwarcd/CZII_to_MQTT
//
//   Sketch to connect to Carrier Comfort Zone II (CZII) RS485 serial bus and send data to
//   and from a MQTT feed
//
//   Uses a MAX485 RS-485 TTL to RS485 MAX485CSA Converter Module For Arduino
//      CZII RS+ = B+
//      CZII RS- = A-
//
//   Must use ESP8266 Arduino from:
//      https://github.com/esp8266/Arduino

#include <ESP8266HTTPUpdateServer.h>
#include <ESP8266mDNS.h>
#include <ESP8266WiFi.h>
#include <Adafruit_MQTT.h>
#include <Adafruit_MQTT_Client.h>
#include <SoftwareSerial.h>
#include "WiFiManager.h"

#include "RingBuffer.h"
#include "ComfortZoneII.h"
#include "Util.h"

// WIFI SETUP
#define WLAN_SSID         "YOUR_SSID"           //WiFi SSID here
#define WLAN_PASS         "YOUR_PASSWORD"       //WiFi password here

// Create an ESP8266 WiFiClient class to connect to the MQTT server.
WiFiClient client;                              // or... use WiFiClientSecure for SSL

// MQTT SETUP
#define MQTT_SERVER       "MQTT_SERVER_URL"     // Local OpenHab and MQ server
#define MQTT_SERVERPORT   1883                  // use 8883 for SSL
#define MQTT_USERNAME     ""                    // MQTT server username
#define MQTT_PASSWORD     ""				            // MQTT server password

const char*              hostName = "CZII";     //
ESP8266WebServer         httpServer(80);        // Http server we will be providing
ESP8266HTTPUpdateServer  httpUpdater(false);    // A OverTheAir update service.

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_Client mqtt(&client, MQTT_SERVER, MQTT_SERVERPORT, MQTT_USERNAME, MQTT_PASSWORD);

// Setup 'czii' (Comfort Zone II) feeds for publishing.
Adafruit_MQTT_Publish zone_mqtt_feed = Adafruit_MQTT_Publish(&mqtt, "czii/zone");
Adafruit_MQTT_Publish status_mqtt_feed = Adafruit_MQTT_Publish(&mqtt, "czii/status");

// Setup 'czii/zonetemp' feed for subscribing to zone info changes.
Adafruit_MQTT_Subscribe mqtt_sub_feed = Adafruit_MQTT_Subscribe(&mqtt, "czii/zonetemp");

// RS485 Software Serial
#define SSerialRX         D5                  	// RS485 Serial Receive pin
#define SSerialTX         D6                    // RS485 Serial Transmit pin
#define SSerialTxControl  D3                    // RS485 Direction control
#define RS485Transmit     HIGH
#define RS485Receive      LOW

SoftwareSerial rs485(SSerialRX, SSerialTX, false, 256);

// CZII Configuration
ComfortZoneII CzII((byte)2);

// CZII Commands
#define COMMAND_TIME_PERIOD     10000
#define DEVICE_ADDRESS          99
byte REQUEST_INFO_TEMPLATE[]          = {1, 0, DEVICE_ADDRESS, 0, 3, 0, 0, 11, 0, 255, 255, 0, 0};  // Note: Replace the table and row values, and calc checksum before sending out request
byte SET_ZONE_TEMPERATURE_TEMPLATE[]  = {1, 0, DEVICE_ADDRESS, 0, 19, 0, 0, 12, 0, 1, 16, 76, 76, 76, 76, 76, 76, 76, 76, 68, 68, 68, 68, 68, 68, 68, 68, 255, 255};

byte TABLE1_POLLING_ROWS[] = {6, TABLE1_TEMPS_ROW, TABLE1_TIME_ROW};
byte rowIndex = 0;

unsigned long lastSendTimeMillis = 0;
unsigned long lastPollingTimeMillis = COMMAND_TIME_PERIOD;   // Delay 10 seconds on startup before sending commands
unsigned long lastReceivedMessageTimeMillis = 0;

// Input/output ring buffers
RingBuffer rs485InputBuf;
RingBuffer rs485OutputBuf;
RingBuffer serialInputBuf;
String serialInputByte;

// Forward Declarations
void MQTT_connect();        // Bug workaround for Arduino 1.6.6, it seems to need a function declaration for some reason (only affects ESP8266, likely an arduino-builder bug).
void webServerHandleRoot();
void configModeCallback();

//********************** Sketch Code ******************************/

void setup() {
  pinMode(BUILTIN_LED, OUTPUT);

  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // Setup software serial (rs485)
  pinMode(SSerialTxControl, OUTPUT);
  digitalWrite(SSerialTxControl, RS485Receive);  // Init Transceiver
  rs485.begin(9600);

  Serial.println();
  Serial.println(F("Starting..."));

  mqtt.subscribe(&mqtt_sub_feed);
}

//
//  Main application loop
//
void loop() {
  wifiSetup();
  processMqttInput();
  processRs485InputStream();
  processSerialInputStream();
  sendPollingCommands();
  sendOutputFrame();
}

//
// Setup Wifi
//
void wifiSetup() {
  if (WiFi.status() == WL_CONNECTED) {
    return;
  }

  // Connect to WiFi access point.
  Serial.println();
  Serial.print(F("Connecting to WiFi SSID: "));
  Serial.println(WLAN_SSID);

  WiFiManager wifiManager;
  wifiManager.setAPCallback(configModeCallback);

  //wifiManager.resetSettings();
  
  // Setup an Access point in order to allow network setup
  if (!wifiManager.startConfigPortal(hostName)) {
      Serial.println("Not connected to WiFi but continuing anyway.");
    } else {
      //if you get here you have connected to the WiFi
      Serial.println("Connected to WiFi.");

        //read updated parameters
      //strcpy(WUNDERGROUND_API_KEY, custom_mqtt_server.getValue());
    }
    
  //wifiManager.autoConnect( hostName );

  WiFi.mode( WIFI_AP_STA );
  WiFi.softAPdisconnect( true );

  // Establish a connection with our configured access point
  while( WiFi.waitForConnectResult() != WL_CONNECTED )
  {
    WiFi.begin();
  }

  Serial.println(F("WiFi connected"));
  IPAddress ip = WiFi.localIP();
  Serial.println("IP address: " + ip.toString());

  httpServer.on("/",      webServerHandleRoot );

  // Add OTA update service provided by library "/update" command
  httpUpdater.setup( &httpServer );
  httpServer.begin( );

  MDNS.begin( hostName );
  MDNS.addService( "http", "tcp", 80 );
}

//
//  Function to connect and reconnect as necessary to the MQTT server.
//  Should be called in the loop function and it will take care if connecting.
//
void MQTT_connect() {
  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.println();
  Serial.println(F("Connecting to MQTT..."));

  uint8_t retries = 3;
  int8_t ret;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
    digitalWrite(BUILTIN_LED, LOW);  // Flash LED while connecting to WiFi

    Serial.println(mqtt.connectErrorString(ret));
    Serial.println(F("Retrying MQTT connection in 5 seconds..."));

    mqtt.disconnect();
    delay(5000);  // wait 5 seconds
    retries--;
    if (retries == 0) {
      // basically die and wait for WDT to reset me
      while (1);
    }

    digitalWrite(BUILTIN_LED, HIGH);
  }

  if (mqtt.connect() == 0) {
    Serial.println(F("MQTT Connected."));
  }
  else {
    Serial.println(F("MQTT Connection Failed!!!!"));
  }

  Serial.println();
}

//
//  Process input from the MQTT subscription feeds
//
void processMqttInput() {
  // Ensure the connection to the MQTT server is alive (this will make the first
  // connection and automatically reconnect when disconnected).  See the MQTT_connect
  // function definition further below.
  MQTT_connect();

  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(1))) {
    if (subscription == &mqtt_sub_feed) {
      Serial.print(F("mqtt_sub_feed received : "));

      String value = (char *)mqtt_sub_feed.lastread;
      Serial.println(value);
      Zone* zone1 =  CzII.getZone(0);
      Zone* zone2 =  CzII.getZone(1);
      byte zone1HeatSetpoint = zone1->getHeatSetpoint();
      byte zone2HeatSetpoint = zone2->getHeatSetpoint();
      byte zone1CoolSetpoint = zone1->getCoolSetpoint();
      byte zone2CoolSetpoint = zone2->getCoolSetpoint();
      bool sendCommand = false;

      if (value == "1") {
        zone1HeatSetpoint++;
        sendCommand = true;
      }
      else if (value == "0") {
        zone1HeatSetpoint--;
        sendCommand = true;
      }

      if (sendCommand) {
        // Update temperatures
        SET_ZONE_TEMPERATURE_TEMPLATE[11] = zone1CoolSetpoint;
        SET_ZONE_TEMPERATURE_TEMPLATE[12] = zone2CoolSetpoint;
        SET_ZONE_TEMPERATURE_TEMPLATE[19] = zone1HeatSetpoint;
        SET_ZONE_TEMPERATURE_TEMPLATE[20] = zone2HeatSetpoint;
        rs485_EnqueFrame(SET_ZONE_TEMPERATURE_TEMPLATE, array_len(SET_ZONE_TEMPERATURE_TEMPLATE));

        // Request latest data
        REQUEST_INFO_TEMPLATE[9] = 1;   // Table = 1
        REQUEST_INFO_TEMPLATE[10] = 16;
        rs485_EnqueFrame(REQUEST_INFO_TEMPLATE, array_len(REQUEST_INFO_TEMPLATE));
      }
    }
  }
}

//
//  Process input data from the rs485 Serial stream.
//
//  We look for valid CZII data frames, convert to JSON and then send to the MQTT server.
//
void processRs485InputStream() {
  // Process input data
  while (rs485Available() > 0) {
    if (!rs485InputBuf.add((byte)rs485Read())) {
      info_println(F("ERROR: INPUT BUFFER OVERRUN!"));
    }

    if (processInputFrame()) {
      debug_println(F("FOUND GOOD CZII FRAME!"));
    }

    lastReceivedMessageTimeMillis = millis();
  }
}

int rs485Available() {
  return rs485.available();
}

int rs485Read() {
  delay(0);   // So we don't get watchdog resets
  return rs485.read();
}

//
//  Process input data from the Serial stream. This data is converted from ASCII to byte values and
//  written out on the rs485 stream.
//
//  Data is expected to be in the form of string representing byte values:
//       "1.0  99.0  19  0.0.12   0.1.16. 78.77.76.76.76.76.76.76. 68.67.68.68.68.68.68.68. "
//
//       - Valid delimiters between bytes are  ' ', '.', or ','
//       - Each line (frame) must be terminated with a '|', '\n', or '\r'
//       - The checksum and will be automatically calculated and therefore should not be included
//
//      (This can be used to send test message frames to the CZII from an external source via the Serial port)
//
void processSerialInputStream() {
  while (Serial.available() > 0) {
    byte input = Serial.read();
    bool processByteString = false;

    if (input == ' ' || input == '.' || input == ',') // space, dot, or comma = value delimiter
    {
      processOutputByteString();
      continue;
    }
    else if (input == '|' || input == '\n' || input == '\r') // message delimiter: new line, or line feed
    {
      processOutputByteString();
      if ( processSerialInputFrame()) {
        debug_println(F("FOUND GOOD FRAME!"));
      }

      serialInputBuf.reset();
      continue;
    }

    serialInputByte = serialInputByte + String(input - 48);
  }
}

void processOutputByteString() {
  if (serialInputByte.length() != 0) {
    // convert to byte and add to buffer
    byte value = (byte)serialInputByte.toInt();
    serialInputBuf.add(value);
    serialInputByte = "";
  }
}

//
//  Process the bytes received from the serial port.  Calculate checksum before sending data
//  out the rs485 port on the CZII bus.
//
bool processSerialInputFrame()
{
  short bufferLength = serialInputBuf.length();

  // Figure out length of buffer
  if (bufferLength < ComfortZoneII::MIN_MESSAGE_SIZE - 2)
  {
    debug_println("serialInputBuf bufferLength < MIN_MESSAGE_SIZE");
    return false;
  }

  byte source = serialInputBuf.peek(ComfortZoneII::SOURCE_ADDRESS_POS);
  byte destination = serialInputBuf.peek(ComfortZoneII::DEST_ADDRESS_POS);
  byte dataLength = serialInputBuf.peek(ComfortZoneII::DATA_LENGTH_POS);
  byte function = serialInputBuf.peek(ComfortZoneII::FUNCTION_POS);

  debug_println("serialInputBuf: source=" + String(source) + ", destination=" + String(destination) + ", dataLength=" + String(dataLength) + ", function=" + String(function));

  byte frameLength = (byte)(ComfortZoneII::DATA_START_POS + dataLength + 2);

  if (frameLength != (bufferLength + 2))
  {
    debug_println("serialInputBuf: **frameLength != bufferLength" + String(frameLength) + ", bufferLength = " + String(bufferLength));
    return false;
  }

  // Add checksum
  byte checksum1 = frameLength - 2;
  unsigned short crc = ModRTU_CRC(serialInputBuf, checksum1);
  serialInputBuf.add(lowByte(crc));
  serialInputBuf.add(highByte(crc));

  dumpFrame(serialInputBuf);

  rs485_TransmitFrame(serialInputBuf);
  return true;
}

//
//  Send the next queued output frame
//
void sendOutputFrame()
{
  if (rs485OutputBuf.length() == 0 ) {
    return;
  }

  int send_time_diff_ms = millis() - lastSendTimeMillis;
  int last_message_time_diff_ms = millis() - lastReceivedMessageTimeMillis;

  // Try to reduce bus contention by delaying 300 ms since last received message (frame) before
  // we send anything out.
  if (rowIndex == 0 && last_message_time_diff_ms < 300) {
    return;
  }

  if (send_time_diff_ms < 100 ) {
    return;
  }

  rs485_TransmitFrame(rs485OutputBuf);

  lastSendTimeMillis = millis();
}

void rs485_EnqueFrame(byte values[], byte size) {
  if (rs485OutputBuf.length() + size > RingBuffer::MAX_BUFFER_SIZE) {
    info_println("ERROR: rs485_EnqueFrame: skipping frame, rs485OutputBuf not large enough");
    return;
  }

  // update checksum
  byte checksum1 =  size - 2;
  unsigned short crc = ModRTU_CRC(values, checksum1);
  values[checksum1] = lowByte(crc);
  values[checksum1 + 1] = highByte(crc);

  for (byte i = 0; i < size; i++) {
    byte value = values[i];
    rs485OutputBuf.add(value);
  }
}

void rs485_TransmitFrame(RingBuffer& ringBuffer) {
  short bufferLength = ringBuffer.length();
  if (bufferLength == 0) {
    return;
  }

  info_print("OUTPUT: ");

  if (bufferLength < ComfortZoneII::DATA_LENGTH_POS) {
    info_println("rs485_TransmitFrame: not enough data");
    return;
  }

  byte dataLength = ringBuffer.peek(ComfortZoneII::DATA_LENGTH_POS);
  byte frameLength = (byte)(ComfortZoneII::DATA_START_POS + dataLength + 2);

  if (bufferLength < frameLength) {
    info_println("rs485_TransmitFrame: not enough data");
    return;
  }

  dumpFrame(ringBuffer);

  digitalWrite(SSerialTxControl, RS485Transmit);

  int index = 0;
  while (index < frameLength) {
    rs485.write(ringBuffer.read());
    index++;
  }

  digitalWrite(SSerialTxControl, RS485Receive);
}

void sendPollingCommands() {
  int polling_time_diff_ms = millis() - lastPollingTimeMillis;

  // Try to reduce bus contention by delaying 1000ms since last received message (frame) before
  // we send anything out.
  // If it hasn't been at least COMMAND_TIME_PERIOD milliseconds since last command
  // or the last message received time is less than a second then return
  if (polling_time_diff_ms < COMMAND_TIME_PERIOD) {
    return;
  }

  for (int i = 0; i < array_len(TABLE1_POLLING_ROWS); i++) {
    REQUEST_INFO_TEMPLATE[9] = 1;   // Table = 1
    REQUEST_INFO_TEMPLATE[10] = TABLE1_POLLING_ROWS[i];
    rs485_EnqueFrame(REQUEST_INFO_TEMPLATE, array_len(REQUEST_INFO_TEMPLATE));
  }

  lastPollingTimeMillis = millis();
}

//
//  This method detects if the current buffer has a valid data frame.  If none is found the buffer is shifted
//  and we return false.
//
//  Carrier Comfort Zone || (CZII) data frame structure:
//    For more info see: https://github.com/jwarcd/CZII_to_MQTT/wiki/CZII-Serial-Protocol
//    (Note: Similar to the Carrier Infinity protocol: https://github.com/nebulous/infinitude/wiki/Infinity-serial-protocol)
//
//   |-----------------------------------------------------------------------------------|
//   |                                       Frame                                       |
//   |-----------------------------------------------------------------------------------|
//   |                      Header                          |           |                |
//   |-------------------------------------------------------           |                |
//   | 2 bytes | 2 bytes | 1 byte |  2 bytes  | 1 byte      |   Data    |   Checksum     |
//   |-----------------------------------------------------------------------------------|
//   | Dest    | Source  | Data   | Reserved  | Function    |  0-255    |    2 bytes     |
//   | Address | Address | Length |           |             |  bytes    |                |
//   |-----------------------------------------------------------------------------------|
//
//    Example Data: 9 0   1 0   3   0 0 11   0 9 1   213 184
//		Destination	= 9
//		Source 		  = 1
//		Data Length = 3
//		Function 	  = 11         (Read Request)
//		Data 		    = 0 9 1      (Table 9, Row 1)
//		Checksum 	  = 213 184
//
//   CZII Function Codes:
//		6 (0x06) Response
//			 1 Byte Length, Data=0x00 – Seems to be an ACK to a write
//			 Variable Length > 3 bytes – a response to a read request
//		11 (0x0B) Read Request
//			 3 byte Length, Data=Table and row of data to get
//		12 (0x0C) Write Request
//			 Variable Length > 3 bytes
//			 First 3 bytes of data are table and row to write to
//			 Following bytes are data to write
//		21 (0x15) Error
//			 1 Byte Length, Data=0x00
//
bool processInputFrame() {
  digitalWrite(BUILTIN_LED, LOW);  // Flash LED to indicate a frame is being processed

  short bufferLength = rs485InputBuf.length();

  // see if the buffer has at least the minimum size for a frame
  if (bufferLength < ComfortZoneII::MIN_MESSAGE_SIZE ) {
    //debug_println("rs485InputBuf: bufferLength < MIN_MESSAGE_SIZE");
    return false;
  }

  byte source = rs485InputBuf.peek(ComfortZoneII::SOURCE_ADDRESS_POS);
  byte destination = rs485InputBuf.peek(ComfortZoneII::DEST_ADDRESS_POS);
  byte dataLength = rs485InputBuf.peek(ComfortZoneII::DATA_LENGTH_POS);
  byte function = rs485InputBuf.peek(ComfortZoneII::FUNCTION_POS);

  //debug_println("rs485InputBuf: source=" + String(source) + ", destination=" + String(destination) + ", dataLength=" + String(dataLength) + ", function=" + String(function));

  short checksum1Pos = ComfortZoneII::DATA_START_POS + dataLength;
  short checksum2Pos = checksum1Pos + 1;
  short frameLength = checksum2Pos + 1;

  // Make sure we have enough data for this frame
  short frameBufferDiff =  frameLength - bufferLength;
  if (frameBufferDiff > 0 && frameBufferDiff < 30) {
    // Don't have enough data yet, wait for another byte...
    debug_print(".");
    return false;
  }

  debug_println();

  byte checkSum1 = rs485InputBuf.peek(checksum1Pos);
  byte checkSum2 = rs485InputBuf.peek(checksum2Pos);

  unsigned short crc = ModRTU_CRC(rs485InputBuf, checksum1Pos);
  byte high = highByte(crc);
  byte low = lowByte(crc);

  if (checkSum2 != high || checkSum1 != low) {
    info_println(F("CRC failed, shifting buffer..."));
    info_println("rs485InputBuf: checkSum1=" + String(checkSum1) + ", checkSum2=" + String(checkSum2) + ", crc=" + String(crc) + ", high=" + String(high) + ", low=" + String(low));
    rs485InputBuf.dump(bufferLength);
    info_println();
    rs485InputBuf.shift(1);
    return false;
  }

  publishCZIIData(rs485InputBuf);

  rs485InputBuf.shift(frameLength);

  digitalWrite(BUILTIN_LED, HIGH);  // Flash LED while processing frames

  return true;
}

//
//	Publish CZII data to the MQTT feed
//
void publishCZIIData(RingBuffer ringBuffer) {
  info_print("RS485: ");
  dumpFrame(ringBuffer);

  CzII.update(ringBuffer);

  if (CzII.isZoneModified()) {
    CzII.clearZoneModified();

    String output = CzII.toZoneJson();
    info_print("MQTT Zone: length=" + String(output.length()) + ", JSON=");
    info_println(output);
    if (!zone_mqtt_feed.publish(output.c_str())) {     // Publish to MQTT server (openhab)
      info_println(F("zone_mqtt_feed.publish Failed"));
    }
  }

  if (CzII.isStatusModified()) {
    CzII.clearStatusModified();

    String output = CzII.toStatusJson();
    info_print("MQTT Status: length=" + String(output.length()) + ", JSON=");
    info_println(output);
    if (!status_mqtt_feed.publish(output.c_str())) {     // Publish to MQTT server (openhab)
      info_println(F("status_mqtt_feed.publish Failed"));
    }
  }
}

//
//   Debug dump of the current frame including the checksum bytes.  Spaces are inserted for
//   readability between the major sections of the frame.
//
void dumpFrame(RingBuffer ringBuffer) {

  if (ringBuffer.length() == 0)
    return;

  // Destination
  Serial.print(String(ringBuffer.peek(ComfortZoneII::DEST_ADDRESS_POS)) + "." + String(ringBuffer.peek(ComfortZoneII::DEST_ADDRESS_POS + 1)));

  // Source
  Serial.print("  " + String(ringBuffer.peek(ComfortZoneII::SOURCE_ADDRESS_POS)) + "." + String(ringBuffer.peek(ComfortZoneII::SOURCE_ADDRESS_POS + 1)));

  // Data Size
  byte dataLength = ringBuffer.peek(ComfortZoneII::DATA_LENGTH_POS);
  Serial.print("  " + String(dataLength) );

  // Function
  if (dataLength < 10)
  {
    Serial.print(" ");  // add extra space
  }
  byte function = ringBuffer.peek(ComfortZoneII::FUNCTION_POS);
  Serial.print("  " + String(ringBuffer.peek(ComfortZoneII::FUNCTION_POS - 2)) + "." + String(ringBuffer.peek(ComfortZoneII::FUNCTION_POS - 1)) + "." + String(function));

  // Data
  if (function < 10)
  {
    Serial.print(" ");  // add extra space
  }

  delay(0);

  Serial.print("  ");
  short totalDataTextLength = 0;
  for (byte pos = ComfortZoneII::DATA_START_POS; pos < (ComfortZoneII::DATA_START_POS + dataLength); pos++) {
    String text = String(ringBuffer.peek(pos)) + ".";
    totalDataTextLength += text.length();
    Serial.print(text);
  }
  for (byte i = 0; i < (60 - totalDataTextLength); i++) {
    Serial.print(" ");
  }

  // Checksum
  byte crcHighByte = ringBuffer.peek(ComfortZoneII::DATA_START_POS + dataLength);
  byte crcLowByte = ringBuffer.peek(ComfortZoneII::DATA_START_POS + dataLength + 1);
  Serial.print("  " + String(crcHighByte) + "." + String(crcLowByte));

  Serial.println();
}

///////////////////////////////////////////////////////////////////
//
// Web Server section
//
///////////////////////////////////////////////////////////////////
void configModeCallback (WiFiManager *myWiFiManager) {
  Serial.println("Entered config mode");
  Serial.println(WiFi.softAPIP());

  Serial.println(myWiFiManager->getConfigPortalSSID());
}

void webServerHandleRoot()
{
  String message;
  message += hostName;

  message += " \n";
  message += "Commands : /update \n";

  httpServer.send(200, "text/plain", message );
}







