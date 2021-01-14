/*    Simple BLE client application that connects to a PC-60FW Viatom Fingertip 
 *    Pulse Oximeter and relays live SPO2 and pulse readings to your chosen
 *    host over the internet.
 *    
 *    The BLE code is largely copied from the ESP32 BLE Arduino/BLE_client
 *    example (author unknown).
 *    
 *    This particular code is designed to work with the Olimex ESP32-Gateway
 *    board that has a wired Ethernet connection provided by a Microchip LAN8710A.
 *    
 *    Wired ethernet is required as the ESP32 can't run the Wifi radio and
 *    the BLE radio simultaniously.
 *    
 *    If you only have a regular ESP32 with no wired ethernet then change
 *    ETHENABLED define.
 *  
 */

/*
 *   --------- An important note on building using the ESP32-Gateway board
 *   
 *   Because the ESP32 BLE library is HUGE it is necessary to change the
 *   partitioning scheme when uploading the code to the ESP32. Some boards
 *   allow this via options under the tools menu, sadly the default definition
 *   for the Olimex esp32-gateway board doesn't do this.
 *   
 *   So it is necessary to edit the boards.txt file and add the following
 *   text in order to add the partition options. This should be done in the
 *   esp32-gateway section. 
 *   
 *   On Linux my boards.txt file is located in:
 *   ~/.arduino15/packages/esp32/hardware/esp32/1.0.1/boards.txt
 *   
 *   Once this is done you should set the partition scheme to:
 *   'Minimal SPIFFS (Large APPS with OTA)'.
 *   

esp32-gateway.menu.PartitionScheme.default=Default
esp32-gateway.menu.PartitionScheme.default.build.partitions=default
esp32-gateway.menu.PartitionScheme.minimal=Minimal (2MB FLASH)
esp32-gateway.menu.PartitionScheme.minimal.build.partitions=minimal
esp32-gateway.menu.PartitionScheme.no_ota=No OTA (Large APP)
esp32-gateway.menu.PartitionScheme.no_ota.build.partitions=no_ota
esp32-gateway.menu.PartitionScheme.no_ota.upload.maximum_size=2097152
esp32-gateway.menu.PartitionScheme.huge_app=Huge APP (3MB No OTA)
esp32-gateway.menu.PartitionScheme.huge_app.build.partitions=huge_app
esp32-gateway.menu.PartitionScheme.huge_app.upload.maximum_size=3145728
esp32-gateway.menu.PartitionScheme.min_spiffs=Minimal SPIFFS (Large APPS with OTA)
esp32-gateway.menu.PartitionScheme.min_spiffs.build.partitions=min_spiffs
esp32-gateway.menu.PartitionScheme.min_spiffs.upload.maximum_size=1966080
esp32-gateway.menu.PartitionScheme.fatflash=16M Fat
esp32-gateway.menu.PartitionScheme.fatflash.build.partitions=ffat

 */

/*

MIT License

Copyright (c) 2021 Martin Whitaker

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

*/

#include "BLEDevice.h"

/* If you only have a regular ESP32 board (with no wired ethernet) don't
 *  define ETHENABLED. Although no data will be sent to the internet you
 *  can still see the data from the Pulse Oximeter.
 */
#define ETHENABLED  1
#define REMOTEHOST  "yourhost.com"

#ifdef ETHENABLED
#include <ETH.h>
#include <WiFiUdp.h>

static bool eth_connected = false;
WiFiUDP udp;

void WiFiEvent(WiFiEvent_t event)
{
  switch (event) {
    case SYSTEM_EVENT_ETH_START:
      Serial.println("ETH Started");
      //set eth hostname here
      ETH.setHostname("pc-60fwproxy");
      break;
    case SYSTEM_EVENT_ETH_CONNECTED:
      Serial.println("ETH Connected");
      break;
    case SYSTEM_EVENT_ETH_GOT_IP:
      Serial.print("ETH MAC: ");
      Serial.print(ETH.macAddress());
      Serial.print(", IPv4: ");
      Serial.print(ETH.localIP());
      if (ETH.fullDuplex()) {
        Serial.print(", FULL_DUPLEX");
      }
      Serial.print(", ");
      Serial.print(ETH.linkSpeed());
      Serial.println("Mbps");
      eth_connected = true;
      udp.begin(WiFi.localIP(),8765);
      break;
    case SYSTEM_EVENT_ETH_DISCONNECTED:
      Serial.println("ETH Disconnected");
      eth_connected = false;
      break;
    case SYSTEM_EVENT_ETH_STOP:
      Serial.println("ETH Stopped");
      eth_connected = false;
      break;
    default:
      break;
  }
}

#endif

// The remote service we wish to connect to.
static BLEUUID serviceUUID("6e400001-b5a3-f393-e0a9-e50e24dcca9e");

// The characteristic of the remote service we are interested in.
static BLEUUID    charUUID("6e400003-b5a3-f393-e0a9-e50e24dcca9e");

static boolean doConnect = false;
static boolean connected = false;
static boolean doScan = false;
static BLERemoteCharacteristic* pRemoteCharacteristic;
static BLEAdvertisedDevice* myDevice;
static BLEScan* pBLEScan;

// These bytes always appear before the sat and hr data on 20 byte BLE packets
static const unsigned char searchheader[5] = { 170, 85, 15, 8, 1 };

int havenewdata=0;
int l_sat=0;
int l_hr=0;
char l_device[128];

static void notifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify)
{
  int i;
  if (length==20) {
    // Find searchheader in the data stream
    for(i=0;i<20-5;i++) {
      if (memcmp(pData+i,searchheader,5)==0) {
        if (havenewdata==0) {
          l_sat=pData[i+5];
          l_hr=pData[i+6];
          havenewdata=1;
          strcpy(l_device,myDevice->getAddress().toString().c_str());
        }
      }
    }
  }
}

class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {
  }

  void onDisconnect(BLEClient* pclient) {
    connected = false;
    doConnect = false;
    doScan = false;
    Serial.println("onDisconnect");
    // restartscan();
    // Attempts to gracefully resume have eluded me... given up and now reboot the ESP after the device disconnects
    delay(1000);
    ESP.restart();
  }
};

bool connectToServer() {
    Serial.print("Forming a connection to ");
    Serial.println(myDevice->getAddress().toString().c_str());
    
    BLEClient*  pClient  = BLEDevice::createClient();
    Serial.println(" - Created client");

    pClient->setClientCallbacks(new MyClientCallback());

    // Connect to the remove BLE Server.
    pClient->connect(myDevice); 
    Serial.println(" - Connected to server");

    // Obtain a reference to the service we are after in the remote BLE server.
    BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
    if (pRemoteService == nullptr) {
      Serial.print("Failed to find our service UUID: ");
      Serial.println(serviceUUID.toString().c_str());
      pClient->disconnect();
      return false;
    }
    Serial.println(" - Found our service");

    // Obtain a reference to the characteristic in the service of the remote BLE server.
    pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
    if (pRemoteCharacteristic == nullptr) {
      Serial.print("Failed to find our characteristic UUID: ");
      Serial.println(charUUID.toString().c_str());
      pClient->disconnect();
      return false;
    }
    Serial.println(" - Found our characteristic");

    if(pRemoteCharacteristic->canNotify()) {
      pRemoteCharacteristic->registerForNotify(notifyCallback);
    } else {
      Serial.print("Unable to regiser for notify callback.");
      pClient->disconnect();
      return false;      
    }

    connected = true;
    return true;
}

/**
 * Scan for BLE servers and find the first one that advertises the service we are looking for.
 */
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
 /**
   * Called for each advertising BLE server.
   */
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    Serial.print("BLE Advertised Device found: ");
    Serial.println(advertisedDevice.toString().c_str());

    // We have found a device, let us now see if it contains the service we are looking for.
    if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID)) {

      BLEDevice::getScan()->stop();
      myDevice = new BLEAdvertisedDevice(advertisedDevice);
      doConnect = true;
      doScan = true;

    } // Found our server
  } // onResult
}; // MyAdvertisedDeviceCallbacks

void restartscan() {
  // Retrieve a Scanner and set the callback we want to use to be informed when we
  // have detected a new device.  Specify that we want active scanning and start the
  // scan to run for 5 seconds.
  Serial.println("Starting scan...");  
  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);
  pBLEScan->start(0, false);
}

void setup() {
  Serial.begin(115200);

#ifdef ETHENABLED
  // Have to use these specific pin assignments on the ESP32-Gateway board
  ETH.begin(0, -1, 23, 18, ETH_PHY_LAN8720, ETH_CLOCK_GPIO17_OUT);  
#endif

  Serial.println("Starting PC-60FW Pulse Oximeter proxy...");

  restartscan();

} // End of setup.


// This is the Arduino main loop function.
void loop() {

  // If the flag "doConnect" is true then we have scanned for and found the desired
  // BLE Server with which we wish to connect.  Now we connect to it.  Once we are 
  // connected we set the connected flag to be true.
  if (doConnect == true) {
    if (connectToServer()) {
      Serial.println("We are now connected to the PC-60FW.");
    } else {
      Serial.println("Failed to connect to BLE device. Rebooting");
      delay(1000);
      ESP.restart();      
    }
    doConnect = false;
  }

  if ((!connected)&&(doScan)) {
    delay(500);
    BLEDevice::getScan()->start(0);  // this is just eample to start scan after disconnect, most likely there is better way to do it in arduino
  }

  if (havenewdata) {
    char tempbuf[200];
    sprintf(tempbuf,"dev=%s&sat=%d&hr=%d",l_device,l_sat,l_hr);
    Serial.println(tempbuf);
#ifdef ETHENABLED
/*
 * Note - I've used UDP to send the data here as it's important this code doesn't block.
 * The other option would have been to multi-thread the apllication and use HTTP or similar
 * to send the data in another thread.
 */
    udp.beginPacket(REMOTEHOST,8765);
    udp.printf(tempbuf);
    udp.endPacket();
#endif
    havenewdata=0;
  }
  
  delay(1000); 
} 

