#ifndef MBLE_H
#define MBLE_H

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// Forward declaration — implemented in OH1-CORE-v1.0_01092026.ino
void parse(byte buffer[], int l);

// Nordic UART (same UUIDs as ah-rc-03)
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

#define BLE_RX_BUFFER_MAX 256

#ifndef BLE_DEBUG
#define BLE_DEBUG 0
#endif

BLEServer *pServer = NULL;
BLECharacteristic *pTxCharacteristic;
BLECharacteristic *pRxCharacteristic;
bool deviceConnected = false;

static char bleCmdBuf[BLE_RX_BUFFER_MAX];
static size_t bleCmdLen = 0;

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) override {
    deviceConnected = true;
    Serial.println("BLE Connected");
  }
  void onDisconnect(BLEServer *pServer) override {
    deviceConnected = false;
    bleCmdLen = 0;
    Serial.println("BLE disconnected");
    pServer->getAdvertising()->start();
  }
};

class MyCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) override {
    String raw = pCharacteristic->getValue();
    const uint8_t *data = (const uint8_t *)raw.c_str();
    size_t len = raw.length();

#if BLE_DEBUG
    Serial.print("BLE << ");
    Serial.print(len);
    Serial.println(" bytes");
#endif

    for (size_t i = 0; i < len; i++) {
      char c = (char)data[i];
      if (c == '\0' || c == '\r') continue;

      if (c == '\n') {
        byte buffer[100];
        size_t copyLen = (bleCmdLen < sizeof(buffer)) ? bleCmdLen : sizeof(buffer);
        for (size_t j = 0; j < copyLen; j++) buffer[j] = (byte)bleCmdBuf[j];
        parse(buffer, (int)copyLen);
        bleCmdLen = 0;
      } else if (bleCmdLen < sizeof(bleCmdBuf) - 1) {
        bleCmdBuf[bleCmdLen++] = c;
      } else {
        bleCmdLen = 0;
      }
    }
  }
};

void startBle(char *BLName) {
  BLEDevice::init(BLName);
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);

  pRxCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID_RX,
      BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  pRxCharacteristic->setCallbacks(new MyCallbacks());

  pTxCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID_TX,
      BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ);
  pTxCharacteristic->addDescriptor(new BLE2902());

  pService->start();
  pServer->getAdvertising()->start();

  Serial.print(BLName);
  Serial.println(" started");
}

#endif  // MBLE_H
