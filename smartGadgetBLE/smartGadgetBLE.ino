#include <ArduinoBLE.h>
#include <SPI.h>
#include <WiFiNINA.h>
#include arduino_secrets.h

///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_WIFIPASS;    // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;            // your network key Index number (needed only for WEP)

//Definitions
#define HTTP_TIMEOUT 10000 //ms
#define TX_INTERVAL 60000 //ms
#define BLE_TIMEOUT 20000 //ms

// Global variables for humidity and temperature fetched
float temperatureTop = 25.6;
int humidityTop = 54;
float temperatureBottom = 23.3;
int humidityBottom = 50;
int co2Top = 400; //ppm
int bottomGadgetRSSI = 0;
int bottomGadgetBattery = -1;
int topGadgetRSSI = 0;
int topGadgetBattery = -1;
unsigned long delayTimer;
unsigned long txTimer;
unsigned long bleTimer;

int status = WL_IDLE_STATUS;

char deviceName[] = "nano33iot-001"; //TODO: Fetch UID from microcontroller
char server[] = SECRET_FUNC_APP;    //URL of Azure Function App
char key[] = SECRET_KEY; //Key of Azure Function App

//Initialize BLE
const char* temperatureUuid = "00002234-b38d-4985-720e-0f993a68ee41";
const char* humidityUuid = "00001234-b38d-4985-720e-0f993a68ee41";
const char* bottomGadgetAddress = "cb:8f:75:a9:72:9f";
const char* topGadgetAddress = "e4:1c:4c:00:d9:24";

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

void loop() {

  txTimer = millis();
  getSensorData();
  connectToWifi();
  sendSensorData();
  while(millis() < txTimer + TX_INTERVAL){
    delay(10);
  }
}


void getSensorData(){
  //Get sensor data from BLE Gadget

  Serial.println("Starting up BLE module");
  //Setup for BLE
  if (!BLE.begin()) {
    Serial.println("* Starting Bluetooth® Low Energy module failed!");
  }

  Serial.println("- Discovering peripheral device...");

  BLEDevice peripheral;
  
  // start scanning for peripheral
  BLE.scanForAddress(bottomGadgetAddress);

  bleTimer = millis();
  while(millis() < bleTimer + BLE_TIMEOUT){
    peripheral = BLE.available();

    if (peripheral) {
      // discovered a peripheral
      BLE.stopScan();
      Serial.println("Peripheral found");
      Serial.println("-----------------------");
  
      // print address
      Serial.print("Address: ");
      Serial.println(peripheral.address());
  
      // print the RSSI
      Serial.print("RSSI: ");
      Serial.println(peripheral.rssi());
      bottomGadgetRSSI = peripheral.rssi();

      if (peripheral.connect()) {
        Serial.println("Connected");
      } else {
        Serial.println("Failed to connect!");
        return;
      }

      // discover peripheral attributes
      Serial.println("Discovering attributes ...");
      if (peripheral.discoverAttributes()) {
        Serial.println("Attributes discovered");
      } else {
        Serial.println("Attribute discovery failed!");
        peripheral.disconnect();
        return;
      }
  
      BLEService batteryService = peripheral.service("180f");
  
      if (batteryService) {
        // use the service
        BLECharacteristic batteryLevelCharacterisic = peripheral.characteristic("2a19");
  
        if (batteryLevelCharacterisic) {
          // use the characteristic
        } else {
          Serial.println("Peripheral does NOT have battery level characteristic");
        }
      } else {
        Serial.println("Peripheral does NOT have battery service");
      }
    }
  }

  //Make sure BLE connection is closed before using WIFI
  Serial.println("Stopping BLE Service");
  BLE.stopScan();
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
    client.print("&temperatureTop=");
    client.print(temperatureTop);
    client.print("&temperatureBottom=");
    client.print(temperatureBottom);
    client.print("&humidityTop=");
    client.print(humidityTop);
    client.print("&humidityBottom=");
    client.print(humidityBottom);
    client.print("&co2Top=");
    client.print(co2Top);
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
  while (client.available()) {
    char c = client.read();
    Serial.write(c);
  }

  //Disconnect
  client.stop();
  Serial.println("Client disconnected");
}

void connectToWifi(){
  // attempt to connect to WiFi network:
  BLE.end();

  // Re-initialize the WiFi driver
  // This is currently necessary to switch from BLE to WiFi
  wiFiDrv.wifiDriverDeinit();
  wiFiDrv.wifiDriverInit();
  
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);
    delay(1000);
    printWiFiStatus();
    // wait 10 seconds for connection:
    delay(10000);
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
