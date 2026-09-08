#ifndef COM_RX_H
#define COM_RX_H

// RS485 receive — same binary protocol as ah-rc-03 com.h.
// 'M' = palm IMU accel/gyro, 'F' = per-finger flexion angles.
// Uncomment to echo raw UART bytes to USB Serial (disables frame parsing).
// #define COM_PASSTHROUGH_DEBUG

// Uncomment for periodic raw MPU sample dump (every 50th frame).
// #define COM_IMU_RAW_DEBUG

// #define DEBUG_COM_RAW
// #define DEBUG_COM
// #define DEBUG_COM_M_FRAME

#include "imu_filter.h"
#include "com_out.h"

#define RS485_SERIAL Serial1
#define RS485_RX_PIN 44
#define RS485_TX_PIN 43
#define RS485_BAUD   115200

#define MPU6050_VALUES 'M'

#define FLEXIONVALUES    'F'
#define FLEX_DATA_LENGTH 10
#define NUM_FINGERS      5

static const byte SOH = 0x02;
static const byte EOT = 0x04;

static const int REC_BUFFER_SIZE = 32;
static byte recBuffer[REC_BUFFER_SIZE];
static int lrec = 0;

static unsigned long lastImuFrameMs = 0;
static bool imuDtReady = false;

static void comRxResetBuffer() {
  lrec = 0;
}

// Recovers from a bad frame start. Payload bytes can legitimately equal SOH, so
// dropping the whole buffer can strand a real frame header that has already
// arrived — and a false SOH with a large length byte would otherwise stall the
// parser indefinitely. Discard the leading byte and re-scan for the next SOH,
// which always shortens the buffer and so guarantees forward progress.
static void comRxResync() {
  int start = 1;
  while (start < lrec && recBuffer[start] != SOH) start++;
  if (start >= lrec) {
    lrec = 0;
    return;
  }
  memmove(recBuffer, &recBuffer[start], lrec - start);
  lrec -= start;
}

static void comRxParse();

static void comRxProcessMpuFrame(unsigned long frameRxTimestampMs, int ldata) {
  int16_t accel[3];
  int16_t gyro[3];

  memcpy(&accel[0], &recBuffer[4], 2);
  memcpy(&accel[1], &recBuffer[6], 2);
  memcpy(&accel[2], &recBuffer[8], 2);
  memcpy(&gyro[0], &recBuffer[10], 2);
  memcpy(&gyro[1], &recBuffer[12], 2);
  memcpy(&gyro[2], &recBuffer[14], 2);

  uint8_t crc = 0;
  crc ^= recBuffer[1];
  crc ^= recBuffer[2];
  crc ^= recBuffer[3];
  for (int i = 0; i < ldata; i++) {
    crc ^= recBuffer[4 + i];
  }
  const bool crcOk = (crc == recBuffer[lrec - 2]);

  const float accelMs2X = (accel[0] / 16384.0f) * GRAVITY_MS2;
  const float accelMs2Y = (accel[1] / 16384.0f) * GRAVITY_MS2;
  const float accelMs2Z = (accel[2] / 16384.0f) * GRAVITY_MS2;
  const float gyroRadX = (gyro[0] / 131.0f) * (PI / 180.0f);
  const float gyroRadY = (gyro[1] / 131.0f) * (PI / 180.0f);
  const float gyroRadZ = (gyro[2] / 131.0f) * (PI / 180.0f);

#if defined(COM_IMU_RAW_DEBUG) || defined(DEBUG_COM)
  static uint32_t mpuDbgCount = 0;
  if ((mpuDbgCount++ % 50) == 0) {
    String valuesMpu = "values mpu ";
    valuesMpu += String(accel[0]);
    valuesMpu += ",";
    valuesMpu += String(accel[1]);
    valuesMpu += ",";
    valuesMpu += String(accel[2]);
    valuesMpu += ",";
    valuesMpu += String(gyro[0]);
    valuesMpu += ",";
    valuesMpu += String(gyro[1]);
    valuesMpu += ",";
    valuesMpu += String(gyro[2]);
    valuesMpu += ",crc,";
    valuesMpu += crcOk ? "1" : "0";
    valuesMpu += ",\n";
    sendValues(addTimestampToValues(valuesMpu, frameRxTimestampMs));
  }
#endif

  if (!crcOk) return;

  float dtSec = 0.0f;
  if (imuDtReady) {
    dtSec = (frameRxTimestampMs - lastImuFrameMs) / 1000.0f;
    if (dtSec < 0.001f) dtSec = 0.001f;
    if (dtSec > 0.05f) dtSec = 0.05f;
  } else {
    imuDtReady = true;
  }
  lastImuFrameMs = frameRxTimestampMs;

  if (dtSec <= 0.0f) return;

  imuFilterUpdate(accelMs2X, accelMs2Y, accelMs2Z, gyroRadX, gyroRadY, gyroRadZ, dtSec);

  float qw, qx, qy, qz;
  imuFilterGetQuaternion(qw, qx, qy, qz);

  if (!(isfinite(qw) && isfinite(qx) && isfinite(qy) && isfinite(qz))) {
    imuFilterReset();
    return;
  }

  String valuesQuat = "values quat ";
  // Palm IMU mounting: published w is sign-flipped so client orientation matches hand.
  valuesQuat += String(qw, 6);
  valuesQuat += ",";
  valuesQuat += String(qx, 6);
  valuesQuat += ",";
  valuesQuat += String(qy, 6);
  valuesQuat += ",";
  valuesQuat += String(qz, 6);
  valuesQuat += ",\n";
  sendValues(addTimestampToValues(valuesQuat, frameRxTimestampMs));

  float linAx, linAy, linAz;
  imuFilterGetLinearAccel(accelMs2X, accelMs2Y, accelMs2Z, linAx, linAy, linAz);

  String valuesLinAccel = "values linaccel ";
  valuesLinAccel += String(linAx, 3);
  valuesLinAccel += ",";
  valuesLinAccel += String(linAy, 3);
  valuesLinAccel += ",";
  valuesLinAccel += String(linAz, 3);
  valuesLinAccel += ",\n";
  sendValues(addTimestampToValues(valuesLinAccel, frameRxTimestampMs));
}

// Finger flexion angles, thumb..pinky. The wire carries deci-degrees (1800 =
// 180.0deg) but "values flex" is published in degrees, matching what ah-rc-03
// clients already expect on that key.
// Index..pinky IMU mounts are reversed vs the palm, so those published angles
// are negated so curling increases the reading (thumb is left as-is).
// -1 means the glove could not read that finger, so the last good value is held.
static void comRxProcessFlexFrame(unsigned long frameRxTimestampMs, int ldata) {
  if (ldata != FLEX_DATA_LENGTH) return;

  uint8_t crc = 0;
  crc ^= recBuffer[1];
  crc ^= recBuffer[2];
  crc ^= recBuffer[3];
  for (int i = 0; i < ldata; i++) {
    crc ^= recBuffer[4 + i];
  }
  if (crc != recBuffer[lrec - 2]) return;

  static int16_t lastAngle[NUM_FINGERS] = {};
  static bool haveAngle[NUM_FINGERS] = {};

  // true => publish -degrees (mounting flipped). Index 0 = thumb.
  static const bool FINGER_INVERT[NUM_FINGERS] = {
    true,  // thumb
    true,   // index
    true,   // middle
    true,   // ring
    true,   // pinky
  };

  String valuesFlex = "values flex ";
  for (int i = 0; i < NUM_FINGERS; i++) {
    int16_t angle;
    memcpy(&angle, &recBuffer[4 + (i * 2)], 2);

    if (angle >= 0) {
      lastAngle[i] = angle;
      haveAngle[i] = true;
    }

    if (haveAngle[i]) {
      float deg = lastAngle[i] / 10.0f;
      if (FINGER_INVERT[i]) deg = -deg;
      valuesFlex += String(deg, 2);
    } else {
      valuesFlex += "nan";
    }
    valuesFlex += ",";
  }
  valuesFlex += "\n";
  sendValues(addTimestampToValues(valuesFlex, frameRxTimestampMs));
}

static void comRxOnByte(byte b) {
#ifdef DEBUG_COM_RAW
  char hexStr[4];
  sprintf(hexStr, "%02X ", b);
  Serial.print(hexStr);
  if (b >= 32 && b <= 126) {
    Serial.print("('");
    Serial.print((char)b);
    Serial.print("') ");
  }
#endif

  if (lrec >= REC_BUFFER_SIZE) {
    comRxResync();
  }
  recBuffer[lrec++] = b;
  comRxParse();
}

static void comRxParse() {
#ifdef DEBUG_COM_RAW
  if (lrec > 0) {
    Serial.print("\n[COM RX] Buffer len=");
    Serial.print(lrec);
    Serial.print(": ");
    for (int i = 0; i < lrec && i < 20; i++) {
      char hexStr[4];
      sprintf(hexStr, "%02X ", recBuffer[i]);
      Serial.print(hexStr);
    }
    Serial.println();
  }
#endif

  if (recBuffer[0] != SOH) {
#ifdef DEBUG_COM
    if (lrec > 0) {
      Serial.print("[COM RX] No SOH. First byte: 0x");
      Serial.println(recBuffer[0], HEX);
    }
#endif
    comRxResync();
    return;
  }

#ifdef DEBUG_COM
  Serial.println("[COM RX] Found SOH");
#endif

  if (lrec <= 3) return;

  int lframe = recBuffer[3];

  // A length that cannot fit was not a real header, so resync now rather than
  // waiting for bytes that will never make the frame valid.
  if (lframe + 6 > REC_BUFFER_SIZE) {
#ifdef DEBUG_COM
    Serial.print("[COM RX] Implausible length ");
    Serial.println(lframe);
#endif
    comRxResync();
    return;
  }
#ifdef DEBUG_COM
  Serial.print("[COM RX] Expected length=");
  Serial.print(lframe + 6);
  Serial.print(", actual=");
  Serial.println(lrec);
#endif

  if (lrec < lframe + 6) {
#ifdef DEBUG_COM
    Serial.print("[COM RX] Still filling. Expected ");
    Serial.print(lframe + 6);
    Serial.print(" bytes, got ");
    Serial.println(lrec);
#endif
    return;
  }

  if (recBuffer[lrec - 1] != EOT) {
#ifdef DEBUG_COM
    Serial.print("[COM RX] Missing EOT at pos ");
    Serial.print(lrec - 1);
    Serial.print(", got 0x");
    Serial.println(recBuffer[lrec - 1], HEX);
#endif
    comRxResync();
    return;
  }

  unsigned long frameRxTimestampMs = millis();
  int ldata = recBuffer[3];

#ifdef DEBUG_COM_M_FRAME
  if (recBuffer[2] == MPU6050_VALUES || recBuffer[2] == FLEXIONVALUES) {
    Serial.print("[COM ");
    Serial.print((char)recBuffer[2]);
    Serial.print(" frame ");
    Serial.print(lrec);
    Serial.print(" bytes] ");
    for (int i = 0; i < lrec; i++) {
      if (recBuffer[i] < 16) Serial.print("0");
      Serial.print(recBuffer[i], HEX);
      if (i < lrec - 1) Serial.print(" ");
    }
    Serial.println();
  }
#endif

#ifdef DEBUG_COM
  char info[50];
  sprintf(info, "[COM RX] Valid frame: type=0x%02X, length=%d", recBuffer[2], recBuffer[3]);
  Serial.println(info);
#endif

  if (recBuffer[2] == MPU6050_VALUES) {
    comRxProcessMpuFrame(frameRxTimestampMs, ldata);
  } else if (recBuffer[2] == FLEXIONVALUES) {
    comRxProcessFlexFrame(frameRxTimestampMs, ldata);
  }

  comRxResetBuffer();
}

inline void comRxInit() {
  RS485_SERIAL.begin(RS485_BAUD, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
  RS485_SERIAL.setTimeout(10);
  imuFilterReset();
  imuDtReady = false;
  lastImuFrameMs = 0;
  comRxResetBuffer();
}

inline void comRxPoll() {
#ifdef COM_PASSTHROUGH_DEBUG
  while (RS485_SERIAL.available() > 0) {
    Serial.write((uint8_t)RS485_SERIAL.read());
  }
#else
  while (RS485_SERIAL.available() > 0) {
    comRxOnByte((byte)RS485_SERIAL.read());
  }
#endif
}

#endif  // COM_RX_H
