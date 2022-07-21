#include <ArduinoBLE.h>
#include <SPI.h>
#include <WiFiNINA.h>
#include "arduino_secrets.h"

///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_WIFIPASS;    // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;            // your network key Index number (needed only for WEP)

//Definitions
#define HTTP_TIMEOUT 10000 //ms
#define TX_INTERVAL 300000 //ms
#define BLE_TIMEOUT 20000 //ms
#define WIFI_TIMEOUT 60000 //ms

// Global variables for humidity and temperature fetched
bool top_update = false;
bool bottom_update = false;
float temperatureTop = 255;
float humidityTop = -1;
float temperatureBottom = 255;
float humidityBottom = -1;
int bottomGadgetRSSI = 127;
int bottomGadgetBattery = -1;
int topGadgetRSSI = 127;
int topGadgetBattery = -1;
unsigned long delayTimer;
unsigned long txTimer;
unsigned long bleTimer;
unsigned long wifiTimer;

int status = WL_IDLE_STATUS;

char deviceName[] = "nano33iot-001"; //TODO: Fetch UID from microcontroller
char server[] = SECRET_FUNC_APP;    //URL of Azure Function App
char key[] = SECRET_KEY; //Key of Azure Function App

//Initialize BLE
#define BLE_UUID_BATTERY_SERVICE          "180F"
#define BLE_UUID_BATTERY                  "2A19"
#define BLE_UUID_TEMP_SERVICE             "00002234-B38D-4985-720E-0F993A68EE41"   
#define BLE_UUID_TEMP                     "00002235-B38D-4985-720E-0F993A68EE41"
#define BLE_UUID_HUM_SERVICE              "00001234-B38D-4985-720E-0F993A68EE41"
#define BLE_UUID_HUM                      "00001235-B38D-4985-720E-0F993A68EE41"

//Used devices
#define BOTTOM_GADGET                     "cb:8f:75:a9:72:9f"
#define TOP_GADGET                        "e4:1c:4c:00:d9:24"

void setup() {

  Serial.begin(115200);
  while (!Serial); //Wait for Serial to connect

  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    while (true); // don't continue
  }

  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.print("Please update to latest firmware: ");
    Serial.println(WIFI_FIRMWARE_LATEST_VERSION);
  }

  Serial.println("Setup finished");
}

float byteBuffer2float(byte buf[4]){
  float y;
  uint32_t* const py = (uint32_t*) &y;
  *py = ((uint32_t) buf[3] << 24) |
        ((uint32_t) buf[2] << 16) |
        ((uint32_t) buf[1] << 8) |
        ((uint32_t) buf[0] << 0);
  return y;
}

void loop() {

  txTimer = millis();
  getSensorData();
  if(connectToWifi()){
    sendSensorData();
  }
  while(millis() < txTimer + TX_INTERVAL){
    delay(10);
  }
}

//Reads value from given service and characteristic
bool bleReadInt(BLEDevice peripheral, char serviceUuid[], char charUuid[], int *val){
      BLEService bleService = peripheral.service(serviceUuid);

      if (bleService) {
        Serial.print("BLE Service found: ");
        Serial.println(serviceUuid);
        BLECharacteristic bleCharacteristic = peripheral.characteristic(charUuid);
        if (bleCharacteristic) {
          Serial.print("BLE Characteristic found: ");
          Serial.println(charUuid);
          byte v;
          bleCharacteristic.readValue(v);
          Serial.print("Value read: ");
          Serial.println(v);
          *val = (int)v;
          return true;
        } else {
          Serial.print("Error - characteristic not found: ");
          Serial.println(charUuid);
          return false;
        }
      } else {
        Serial.print("Error - service not found: ");
        Serial.println(serviceUuid);
        return false;
      }
}

//Reads value from given service and characteristic
bool bleReadFloat(BLEDevice peripheral, char serviceUuid[], char charUuid[], float *val){
      BLEService bleService = peripheral.service(serviceUuid);

      if (bleService) {
        Serial.print("BLE Service found: ");
        Serial.println(serviceUuid);
        BLECharacteristic bleCharacteristic = peripheral.characteristic(charUuid);
        if (bleCharacteristic) {
          Serial.print("BLE Characteristic found: ");
          Serial.println(charUuid);
          byte buf[4];
          bleCharacteristic.readValue(buf, 4);
          *val = byteBuffer2float(buf);
          Serial.print("Value read: ");
          Serial.println(*val);
          return true;
        } else {
          Serial.print("Error - characteristic not found: ");
          Serial.println(charUuid);
          return false;
        }
      } else {
        Serial.print("Error - service not found: ");
        Serial.println(serviceUuid);
        return false;
      }
}

//Handles connection to one SHT gadget and readout of data, will fill the data into resp. variables
bool getSHTData(char addr[], float *temp, float *hum, int *sig, int *batt){
  Serial.println("- Discovering peripheral device...");

  BLEDevice peripheral;
  BLE.scanForAddress(addr);

  bleTimer = millis();
  while(millis() < bleTimer + BLE_TIMEOUT){
    peripheral = BLE.available();

    if (peripheral) {
      // discovered a peripheral
      BLE.stopScan(); //Stop scanning (keeps module busy)
      Serial.println("Peripheral found");
      Serial.println("-----------------------");
  
      // print address
      Serial.print("Address: ");
      Serial.println(peripheral.address());
  
      // print the RSSI
      Serial.print("RSSI: ");
      Serial.println(peripheral.rssi());
      *sig = peripheral.rssi();
      
      if (peripheral.connect()) {
        Serial.println("Connected");
      } else {
        Serial.println("Failed to connect!");
        peripheral.disconnect();
        return false;
      }

      delay(100); //Sometimes attribute discovery fails without waiting here
      
      // discover peripheral attributes
      Serial.println("Discovering attributes ...");
      if (peripheral.discoverAttributes()) {
        Serial.println("Attributes discovered");
      } else {
        Serial.println("Attribute discovery failed!");
        peripheral.disconnect();
        return false;
      }

      delay(100);
      
      if(bleReadInt(peripheral, BLE_UUID_BATTERY_SERVICE, BLE_UUID_BATTERY, batt) &
         bleReadFloat(peripheral, BLE_UUID_TEMP_SERVICE, BLE_UUID_TEMP, temp) & 
         bleReadFloat(peripheral, BLE_UUID_HUM_SERVICE, BLE_UUID_HUM, hum)){
        peripheral.disconnect();
        return true;
      }
    }
  }

  //Make sure BLE connection is closed before using WIFI
  peripheral.disconnect();
  return false;
}


void getSensorData(){
  //Get data from all sensor
  WiFi.end();
  Serial.println("Starting up BLE module");
  //Setup for BLE
  if (!BLE.begin()) {
    Serial.println("* Starting Bluetooth® Low Energy module failed!");
  }

  //Read top Gadget
  if (getSHTData(TOP_GADGET, &temperatureTop, &humidityTop, &topGadgetRSSI, &topGadgetBattery)){
    top_update = true;
  }

  //Read bottom Gadget
  if (getSHTData(BOTTOM_GADGET, &temperatureBottom, &humidityBottom, &bottomGadgetRSSI, &bottomGadgetBattery)){
    bottom_update = true;
  }

  BLE.end();
}


void sendSensorData(){
  WiFiSSLClient client;
  
  //Sending GET request to client
  Serial.println("\nStarting connection to server...");
  if (client.connect(server, 443)) {
    // Make a HTTP request:
    client.print("GET /api/sensor_handler");
    client.print("?code=");
    client.print(key);
    client.print("&deviceName=");
    client.print(deviceName);
    if(top_update){
      client.print("&batteryTop=");
      client.print(topGadgetBattery);
      client.print("&rssiTop=");
      client.print(topGadgetRSSI);
      client.print("&temperatureTop=");
      client.print(temperatureTop);
      client.print("&humidityTop=");
      client.print(humidityTop);
    }
    if(bottom_update){
      client.print("&batteryBottom=");
      client.print(bottomGadgetBattery);
      client.print("&rssiBottom=");
      client.print(bottomGadgetRSSI);
      client.print("&temperatureBottom=");
      client.print(temperatureBottom);
      client.print("&humidityBottom=");
      client.print(humidityBottom);
    }
    //Hum and temp fields go here afterwards, separated by "&"
    client.println(" HTTP/1.1");
    
    client.print("Host: ");
    client.println(server);

    client.print("Accept: */*");

    client.println("Connection: close");
    client.println();
  }

  //Wait until data becomes available
  Serial.println("Waiting for answer");
  delayTimer = millis();
  while(!client.available()){
    delay(10);
    if (millis() > delayTimer + HTTP_TIMEOUT){
      Serial.println("HTTP Timeout reached");
      break;
    }
  }

  //Read answer from server but do not consider for this program
  bool server_answer = false;
  while (client.available()) {
    server_answer = true;
    char c = client.read();
    Serial.write(c);
  }

  //Disconnect
  client.stop();
  Serial.println("Client disconnected");

  if(server_answer){
    bottom_update = false;
    top_update = false;
  }
  WiFi.end();
}

bool connectToWifi(){
  // attempt to connect to WiFi network:
  BLE.end();
  delay(50);
  // Re-initialize the WiFi driver
  // This is currently necessary to switch from BLE to WiFi
  wiFiDrv.wifiDriverDeinit();
  wiFiDrv.wifiDriverInit();
  status = 0;

  wifiTimer = millis();
  while (status != WL_CONNECTED) {
    delay(50);
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);
    Serial.print("Wifi status: ");
    Serial.println(status);
    delay(1000);
    printWiFiStatus();
    if(status == WL_CONNECTED) {
      delay(10000); //Wait for connection
      return true;
    }
    else if(millis() > wifiTimer + WIFI_TIMEOUT){
      Serial.println("WIFI Timeout reached");
      return false;
    }
    wiFiDrv.wifiDriverDeinit();
    wiFiDrv.wifiDriverInit();
  }
}

void printWiFiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());
  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);
  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}
