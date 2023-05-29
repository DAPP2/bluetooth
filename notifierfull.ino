//server
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <BLEServer.h>
#include <Wire.h>
#include <Arduino.h>
#include <vector>
#include <string.h>
#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
#include <freertos/FreeRTOS.h>

//BLE Server name (the other ESP32 name running the server sketch)
#define bleServerName "Navi"
#define SERIAL_PORT Serial

#define WIRE_PORT Wire // Your desired Wire port.      Used when "USE_SPI" is not defined
// The value of the last bit of the I2C address.
// On the SparkFun 9DoF IMU breakout the default is 1, and when the ADR jumper is closed the value becomes 0
#define AD0_VAL 1

ICM_20948_I2C myICM; // Otherwise create an ICM_20948_I2C object

BLECharacteristic *pCharacteristic;
bool deviceConnected = false;
char* msg = "Hello Charlie";
/*std::vector<int> integerList;

    for (int i = 0; i < msg.length(); i++) {
        char c = input[i];
    int intValue = static_cast<int>(c);
    integerList.push_back(intValue);
}*/

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    deviceConnected = true;
  };

  void onDisconnect(BLEServer *pServer) {
    deviceConnected = false;
  }
};

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  SERIAL_PORT.begin(115200);
  while (!SERIAL_PORT)
  {
  };
  WIRE_PORT.begin(5,4);
  WIRE_PORT.setClock(400000);

  bool initialized = false;
  while (!initialized)
  {


    myICM.begin(WIRE_PORT, AD0_VAL);


    SERIAL_PORT.print(F("Initialization of the sensor returned: "));
    SERIAL_PORT.println(myICM.statusString());
    if (myICM.status != ICM_20948_Stat_Ok)
    {
      SERIAL_PORT.println("Trying again...");
      delay(200);
    }
    else
    {
      initialized = true;
    }
  }
  //OLED display setup

  Wire.begin();

  BLEDevice::init(bleServerName);
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(BLEUUID((uint16_t)0x181A));  // Environmental Sensing
  pCharacteristic = pService->createCharacteristic(
    BLEUUID((uint16_t)0x2A59),  // Analog Output
    BLECharacteristic::PROPERTY_NOTIFY);
  pCharacteristic->addDescriptor(new BLE2902());

  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(pService->getUUID());
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x0);
  pAdvertising->setMinPreferred(0x1F);
  BLEDevice::startAdvertising();
}
/****************************************************************
 * Example1_Basics.ino
 * ICM 20948 Arduino Library Demo
 * Use the default configuration to stream 9-axis IMU data
 * Owen Lyke @ SparkFun Electronics
 * Original Creation Date: April 17 2019
 *
 * Please see License.md for the license information.
 *
 * Distributed as-is; no warranty is given.
 ***************************************************************/


//#define USE_SPI       // Uncomment this to use SPI

void printFormattedFloat(float val, uint8_t leading, uint8_t decimals)
{
  float aval = abs(val);
  if (val < 0)
  {
    SERIAL_PORT.print("-");
  }
  else
  {
    SERIAL_PORT.print(" ");
  }
  if (val < 0)
  {
    SERIAL_PORT.print(-val, decimals);
  }
  else
  {
    SERIAL_PORT.print(val, decimals);
  }
}

void printScaledAGMT(ICM_20948_I2C *sensor){
  SERIAL_PORT.print("Scaled. Acc (mg) [ ");
  printFormattedFloat(sensor->accX(), 5, 2);
  sendRaws(sensor->accX(), 5, 2);
  delay(1000);
  SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->accY(), 5, 2);
  sendRaws(sensor->accY(), 5, 2);
  delay(1000);
  SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->accZ(), 5, 2);
  sendRaws(sensor->accZ(), 5, 2);
  delay(1000);
  SERIAL_PORT.print(" ], Gyr (DPS) [ ");
  printFormattedFloat(sensor->gyrX(), 5, 2);
  sendRaws(sensor->gyrX(), 5, 2);
  delay(1000);
  SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->gyrY(), 5, 2);
  sendRaws(sensor->gyrY(), 5, 2);
  delay(1000);
  SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->gyrZ(), 5, 2);
  sendRaws(sensor->gyrZ(), 5, 2);
  delay(1000);
  SERIAL_PORT.print(" ], Mag (uT) [ ");
  printFormattedFloat(sensor->magX(), 5, 2);
  sendRaws(sensor->magX(), 5, 2);
  delay(1000);
  SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->magY(), 5, 2);
  sendRaws(sensor->magY(), 5, 2);
  delay(1000);
  SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->magZ(), 5, 2);
  sendRaws(sensor->magZ(), 5, 2);
  delay(1000);
  SERIAL_PORT.print(" ], Tmp (C) [ ");
  printFormattedFloat(sensor->temp(), 5, 2);
  sendRaws(sensor->temp(), 5, 2);
  SERIAL_PORT.print(" ]");
  SERIAL_PORT.println();
}


void sendRaws(float val, uint8_t leading, uint8_t decimals){
    int uintValue = static_cast<int>(val*1000);
    pCharacteristic->setValue(uintValue);
    pCharacteristic->notify();
}

void loop() {
  if (deviceConnected) {
  if (myICM.dataReady())
  {
    myICM.getAGMT();         // The values are only updated when you call 'getAGMT'
                             //    printRawAGMT( myICM.agmt );     // Uncomment this to see the raw values, taken directly from the agmt structure
    printScaledAGMT(&myICM); // This function takes into account the scale settings from when the measurement was made to calculate the values with units
  }
  else
  {
    SERIAL_PORT.println("Waiting for data");
  }
  delay(50);
  }
}
