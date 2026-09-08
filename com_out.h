#ifndef COM_OUT_H
#define COM_OUT_H

// Requires mBle.h included first (deviceConnected, pTxCharacteristic)

bool txValues = true;

void sendValues(String values) {
  if (!txValues) return;

  Serial.print(values);

  if (deviceConnected && pTxCharacteristic != nullptr) {
    pTxCharacteristic->setValue(values.c_str());
    pTxCharacteristic->notify();
  }
}

String addTimestampToValues(String values, unsigned long timestampMs) {
  if (values.endsWith("\n")) {
    values.remove(values.length() - 1);
  }
  if (!values.endsWith(",")) {
    values += ",";
  }
  values += "ts,";
  values += String(timestampMs);
  values += ",\n";
  return values;
}

#endif  // COM_OUT_H
