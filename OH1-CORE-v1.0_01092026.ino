#include <SimpleFOC.h>
#include "mBle.h"
#include "com_out.h"
#include "com_rx.h"

static const char *FIRMWARE_VERSION = "OH1-CORE-v1.0_01092026 - unified FOC + RS485 IMU";
static const unsigned long STREAM_MS = 100;

static const int NUM_MOTORS = 5;
static const int POLE_PAIRS = 11;            // GMT4108 / GM4108H 24N22P
static const float PHASE_RESISTANCE = 5.55f; // GM4108H-120T 11.1Ω L-L → ~5.55Ω per phase (star)
static const float MOTOR_KV = 55.0f;         // ~1100 rpm @ 20 V

// ---------- Motor PWM + enable ----------
// Motor#  Driver  Ref   IN1/2/3          EN     Connector
// 1       A       U5    4,5,6            7      P1
// 2       D       U6    12,13,14         21     P2
// 3       B       U7    15,16,17         18     P3
// 4       E       U8    38,39,40         41     P4  (LEDC; MCPWM max 4×3PWM)
// 5       C       U9    8,9,10           11     P5
static const int PWM_PINS[NUM_MOTORS][3] = {
  {4,  5,  6},    // M1
  {12, 13, 14},   // M2
  {15, 16, 17},   // M3
  {38, 39, 40},   // M4 — LEDC
  {8,  9,  10},   // M5
};
static const int EN_PINS[NUM_MOTORS] = {7, 21, 18, 41, 11};
static const char *DRIVER_REF[NUM_MOTORS] = {"U5/A/P1", "U6/D/P2", "U7/B/P3", "U8/E/P4", "U9/C/P5"};

// M4 is driven by LEDC because the ESP32 MCPWM peripheral only supports 4× 3PWM.
static const int LEDC_MOTOR_INDEX = 3;

// Shared nFAULT bus (active-low, open-drain OR of all drivers)
static const int FAULT_PIN = 47;
static const unsigned long DRIVER_ENABLE_STAGGER_MS = 200;

// ---------- AS5600s via TCA9548A ----------
static const int I2C_HOST_SDA = 1;
static const int I2C_HOST_SCL = 2;
static uint8_t muxAddr = 0x70;
static const int MUX_CHANNELS[NUM_MOTORS] = {0, 1, 2, 3, 4};

// Mechanical mounting: these motors are fitted reversed, so +fwd and rising
// angle must both flip for them. Combined with the FOC sensor_direction in
// commandSign(), "fwd" always increases the reported angle on every motor.
static const bool MOTOR_INVERT[NUM_MOTORS] = {false, true, false, true, true};

// ---------- Limits ----------
static const float DRIVER_BUS_VOLTAGE = 12.0f;
static const float OPENLOOP_VOLTAGE_LIMIT = 1.5f;  // fallback only, stalls over-current if raised
static const float CLOSED_LOOP_VOLTAGE_LIMIT = 6.0f;
static const float MOTOR_CURRENT_LIMIT = 0.6f;     // amps (estimated from phase resistance)
static const float ALIGN_VOLTAGE = 4.0f;
static const float DEMO_SPEED = 2.0f;              // rad/s
static const float MAX_VELOCITY = 3.0f;            // rad/s
static const unsigned long DEMO_MOVE_MS = 2000;
static const float MIN_TRAVEL_RAD = 0.05f;         // reject tiny min/max span
static const unsigned long MAX_ENERGISE_MS = 30000;
static const unsigned long POSITION_TIMEOUT_MS = 10000;
static const unsigned long TORQUE_TIMEOUT_MS = 10000;
static const float POSITION_TOLERANCE_RAD = 0.02f;
static const unsigned long POSITION_SETTLE_MS = 300;
static const unsigned long TRACE_MS = 200;

// ---------- Closed-loop gains (velocity PID outputs amps in current mode) ----------
static const float VEL_P = 0.10f;
static const float VEL_I = 1.00f;
static const float VEL_D = 0.0f;
static const float VEL_LPF_TF = 0.05f;
static const float ANGLE_P = 6.0f;

// ---------- M4 LEDC ----------
static const int LEDC_RES_BITS = 10;
static const int LEDC_FREQ_HZ = 25000;
static const int LEDC_MAX_DUTY = (1 << LEDC_RES_BITS) - 1;

// Declared before the first function so the Arduino auto-prototype for
// modeName(CtrlMode) doesn't land above this definition.
enum CtrlMode : uint8_t { CTRL_IDLE = 0, CTRL_VELOCITY, CTRL_POSITION, CTRL_TORQUE };

void beginHostI2C() {
  Wire.begin(I2C_HOST_SDA, I2C_HOST_SCL, 400000UL);
}

bool selectMuxChannel(int channel) {
  Wire.beginTransmission(muxAddr);
  Wire.write(1 << channel);
  uint8_t err = Wire.endTransmission();
  delayMicroseconds(200);
  return err == 0;
}

bool as5600Present() {
  Wire.beginTransmission(0x36);
  return Wire.endTransmission() == 0;
}

// ---------- LEDC driver so M4 can use the same SimpleFOC stack as M1/M2/M3/M5 ----------
class LedcDriver3PWM : public BLDCDriver {
public:
  LedcDriver3PWM(int phA, int phB, int phC, int en)
    : pwmA(phA), pwmB(phB), pwmC(phC), enablePin(en) {
    voltage_power_supply = DRIVER_BUS_VOLTAGE;
    voltage_limit = NOT_SET;
    pwm_frequency = LEDC_FREQ_HZ;
  }

  int init() override {
    pinMode(enablePin, OUTPUT);
    digitalWrite(enablePin, !enable_active_high);
    if (!_isset(voltage_limit) || voltage_limit > voltage_power_supply) {
      voltage_limit = voltage_power_supply;
    }
    const int pins[3] = {pwmA, pwmB, pwmC};
    for (int i = 0; i < 3; i++) {
      if (!ledcAttach(pins[i], LEDC_FREQ_HZ, LEDC_RES_BITS)) {
        initialized = false;
        return 0;
      }
    }
    initialized = true;
    setPwm(0, 0, 0);
    return 1;
  }

  void enable() override {
    digitalWrite(enablePin, enable_active_high);
    setPwm(0, 0, 0);
  }

  void disable() override {
    setPwm(0, 0, 0);
    digitalWrite(enablePin, !enable_active_high);
  }

  void setPwm(float Ua, float Ub, float Uc) override {
    Ua = _constrain(Ua, 0.0f, voltage_limit);
    Ub = _constrain(Ub, 0.0f, voltage_limit);
    Uc = _constrain(Uc, 0.0f, voltage_limit);
    dc_a = _constrain(Ua / voltage_power_supply, 0.0f, 1.0f);
    dc_b = _constrain(Ub / voltage_power_supply, 0.0f, 1.0f);
    dc_c = _constrain(Uc / voltage_power_supply, 0.0f, 1.0f);
    ledcWrite(pwmA, (uint32_t)(dc_a * LEDC_MAX_DUTY));
    ledcWrite(pwmB, (uint32_t)(dc_b * LEDC_MAX_DUTY));
    ledcWrite(pwmC, (uint32_t)(dc_c * LEDC_MAX_DUTY));
  }

  // 3PWM with a single shared enable — no per-phase high-impedance control
  void setPhaseState(PhaseState, PhaseState, PhaseState) override {}

  int pwmA, pwmB, pwmC, enablePin;
};

// ---------- AS5600 via TCA9548A: selects the mux channel before every read ----------
class MuxMagneticSensorI2C : public MagneticSensorI2C {
public:
  MuxMagneticSensorI2C(MagneticSensorI2CConfig_s config, int motorIndex)
    : MagneticSensorI2C(config), motorIndex_(motorIndex) {}

  void init(TwoWire *w = &Wire) {
    selectMuxChannel(MUX_CHANNELS[motorIndex_]);
    MagneticSensorI2C::init(w);
  }

  float getSensorAngle() override {
    if (!selectMuxChannel(MUX_CHANNELS[motorIndex_])) return 0;
    return MagneticSensorI2C::getSensorAngle();
  }

private:
  int motorIndex_;
};

MuxMagneticSensorI2C focSensors[NUM_MOTORS] = {
  MuxMagneticSensorI2C(AS5600_I2C, 0),
  MuxMagneticSensorI2C(AS5600_I2C, 1),
  MuxMagneticSensorI2C(AS5600_I2C, 2),
  MuxMagneticSensorI2C(AS5600_I2C, 3),
  MuxMagneticSensorI2C(AS5600_I2C, 4),
};

BLDCDriver3PWM driverM1(PWM_PINS[0][0], PWM_PINS[0][1], PWM_PINS[0][2], EN_PINS[0]);
BLDCDriver3PWM driverM2(PWM_PINS[1][0], PWM_PINS[1][1], PWM_PINS[1][2], EN_PINS[1]);
BLDCDriver3PWM driverM3(PWM_PINS[2][0], PWM_PINS[2][1], PWM_PINS[2][2], EN_PINS[2]);
LedcDriver3PWM driverM4(PWM_PINS[3][0], PWM_PINS[3][1], PWM_PINS[3][2], EN_PINS[3]);
BLDCDriver3PWM driverM5(PWM_PINS[4][0], PWM_PINS[4][1], PWM_PINS[4][2], EN_PINS[4]);

BLDCDriver *driverFor[NUM_MOTORS] = {&driverM1, &driverM2, &driverM3, &driverM4, &driverM5};

BLDCMotor motors[NUM_MOTORS] = {
  BLDCMotor(POLE_PAIRS, PHASE_RESISTANCE, MOTOR_KV),
  BLDCMotor(POLE_PAIRS, PHASE_RESISTANCE, MOTOR_KV),
  BLDCMotor(POLE_PAIRS, PHASE_RESISTANCE, MOTOR_KV),
  BLDCMotor(POLE_PAIRS, PHASE_RESISTANCE, MOTOR_KV),
  BLDCMotor(POLE_PAIRS, PHASE_RESISTANCE, MOTOR_KV),
};

bool sensorOk[NUM_MOTORS] = {};
bool motorReady[NUM_MOTORS] = {};
bool focReady[NUM_MOTORS] = {};
bool faultLatched = false;
int pwm50Motor = -1;  // motor index holding 50% duty test, or -1
unsigned long motorEnergiseStartMs[NUM_MOTORS] = {};
bool motorEnergised[NUM_MOTORS] = {};

// Raw (unwrapped) sensor angle + manual min/max calibration in the logical frame
float continuousAngle[NUM_MOTORS] = {};
float angleMin[NUM_MOTORS] = {};
float angleMax[NUM_MOTORS] = {};
bool haveMin[NUM_MOTORS] = {};
bool haveMax[NUM_MOTORS] = {};
bool calibrated[NUM_MOTORS] = {};

CtrlMode ctrlMode[NUM_MOTORS] = {};
float ctrlTarget[NUM_MOTORS] = {};        // logical frame: rad/s, rad, or amps
unsigned long ctrlEndMs[NUM_MOTORS] = {}; // 0 = run indefinitely
bool ctrlHold[NUM_MOTORS] = {};
unsigned long posInToleranceMs[NUM_MOTORS] = {};

bool traceActive[NUM_MOTORS] = {};
unsigned long nextTraceMs[NUM_MOTORS] = {};
float traceStartAngle[NUM_MOTORS] = {};
bool focDebug = false;

int demoIndex = -1;
bool demoStepStarted = false;

String inputBuffer = "";

// ---------- Frames ----------
// raw     : encoder angle as read
// logical : what the user sees and what min/max store (raw flipped for reversed mounts)
// shaft   : SimpleFOC's frame (sensor_direction * raw)
float motorAngle(int i) {
  return MOTOR_INVERT[i] ? -continuousAngle[i] : continuousAngle[i];
}

// Converts a logical command (velocity, angle or torque) into SimpleFOC's shaft frame.
float commandSign(int i) {
  float s = MOTOR_INVERT[i] ? -1.0f : 1.0f;
  if (motors[i].sensor_direction == Direction::CCW) s = -s;
  return s;
}

bool probeMux() {
  beginHostI2C();
  delay(50);
  for (int attempt = 0; attempt < 5; attempt++) {
    for (uint8_t addr = 0x70; addr <= 0x77; addr++) {
      Wire.beginTransmission(addr);
      if (Wire.endTransmission() == 0) {
        muxAddr = addr;
        return true;
      }
    }
    delay(20);
  }
  return false;
}

float updateContinuousAngle(int i) {
  if (!sensorOk[i]) return continuousAngle[i];
  focSensors[i].update();
  continuousAngle[i] = focSensors[i].getAngle();
  return continuousAngle[i];
}

void refreshCalibrated(int i) {
  if (!haveMin[i] || !haveMax[i]) {
    calibrated[i] = false;
    return;
  }
  // Allow setting ends in either encoder order
  if (angleMax[i] < angleMin[i]) {
    float tmp = angleMin[i];
    angleMin[i] = angleMax[i];
    angleMax[i] = tmp;
  }
  calibrated[i] = (angleMax[i] - angleMin[i]) >= MIN_TRAVEL_RAD;
}

float normalizedPosition(int i) {
  if (!calibrated[i]) return NAN;
  float span = angleMax[i] - angleMin[i];
  if (span < MIN_TRAVEL_RAD) return NAN;
  return _constrain((motorAngle(i) - angleMin[i]) / span, 0.0f, 1.0f);
}

bool initOneSensor(int i) {
  sensorOk[i] = false;
  if (!selectMuxChannel(MUX_CHANNELS[i])) return false;
  focSensors[i].init(&Wire);
  beginHostI2C();  // MagneticSensorI2C::init() re-runs Wire.begin() with default pins
  if (!selectMuxChannel(MUX_CHANNELS[i])) return false;
  if (!as5600Present()) return false;
  focSensors[i].update();
  continuousAngle[i] = focSensors[i].getAngle();
  sensorOk[i] = true;
  return true;
}

void printAllSensors() {
  Serial.print("sensors:");
  for (int i = 0; i < NUM_MOTORS; i++) {
    Serial.print(" M");
    Serial.print(i + 1);
    Serial.print('=');
    if (!selectMuxChannel(MUX_CHANNELS[i]) || !as5600Present()) {
      sensorOk[i] = false;
      Serial.print('?');
      continue;
    }
    sensorOk[i] = true;
    updateContinuousAngle(i);
    Serial.print(motorAngle(i) * 180.0f / PI, 1);
    Serial.print("deg");
    if (calibrated[i]) {
      Serial.print("/pos=");
      Serial.print(normalizedPosition(i), 3);
    } else if (haveMin[i] || haveMax[i]) {
      Serial.print("/partial");
    }
  }
  Serial.println();
}

// Probe every TCA9548A channel for an AS5600 @ 0x36
void muxScan() {
  Serial.print("mux @0x");
  Serial.print(muxAddr, HEX);
  Serial.println(" channel scan (AS5600):");
  for (int ch = 0; ch < 8; ch++) {
    Serial.print("  ch");
    Serial.print(ch);
    Serial.print(": ");
    if (!selectMuxChannel(ch)) {
      Serial.println("mux NACK");
      continue;
    }
    Serial.println(as5600Present() ? "AS5600 OK" : "no device");
  }
  Serial.print("Mapped M1..M5 → ch ");
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (i) Serial.print(",");
    Serial.print(MUX_CHANNELS[i]);
  }
  Serial.println();
}

// ---------- Driver / motor power ----------
// motorReady[] is only set once initMotor() returns, so power helpers key off the
// driver's own initialized flag — they are used during init too.
bool driverUsable(int motorIndex) {
  return driverFor[motorIndex]->initialized;
}

void motorCoast(int motorIndex) {
  if (driverUsable(motorIndex)) motors[motorIndex].setPhaseVoltage(0, 0, 0);
  motorEnergised[motorIndex] = false;
}

void markMotorEnergised(int motorIndex) {
  if (!motorEnergised[motorIndex]) {
    motorEnergised[motorIndex] = true;
    motorEnergiseStartMs[motorIndex] = millis();
  }
}

void motorDisable(int motorIndex) {
  if (!driverUsable(motorIndex)) return;
  motorCoast(motorIndex);
  motors[motorIndex].disable();
}

void motorEnable(int motorIndex) {
  if (!driverUsable(motorIndex)) {
    Serial.print("M");
    Serial.print(motorIndex + 1);
    Serial.println(" driver not initialised");
    return;
  }
  if (faultLatched) {
    Serial.println("FAULT latched — clear with 'fault clear' before enabling");
    return;
  }
  motors[motorIndex].enable();
  motorCoast(motorIndex);
}

void stopMotor(int i, const char *why) {
  bool wasActive = ctrlMode[i] != CTRL_IDLE;
  ctrlMode[i] = CTRL_IDLE;
  ctrlEndMs[i] = 0;
  ctrlHold[i] = false;
  posInToleranceMs[i] = 0;
  if (motorReady[i]) {
    motors[i].PID_velocity.reset();
    motors[i].P_angle.reset();
    motors[i].target = 0;
    motorCoast(i);
  }
  if (wasActive && traceActive[i]) {
    Serial.print("M");
    Serial.print(i + 1);
    Serial.print(" stop (");
    Serial.print(why ? why : "");
    Serial.print(") ang=");
    Serial.print(motorAngle(i) * 180.0f / PI, 1);
    Serial.print(" d=");
    Serial.println((motorAngle(i) - traceStartAngle[i]) * 180.0f / PI, 1);
  }
  traceActive[i] = false;
}

void stopAllMotors(const char *why) {
  for (int i = 0; i < NUM_MOTORS; i++) stopMotor(i, why);
}

void motorDisableAll() {
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (motorReady[i]) motorDisable(i);
  }
}

void checkMotorEnergiseTimeout() {
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (!motorEnergised[i]) continue;
    // An explicit position hold is allowed to stay energised indefinitely
    if (ctrlMode[i] == CTRL_POSITION && ctrlHold[i]) continue;
    if (millis() - motorEnergiseStartMs[i] < MAX_ENERGISE_MS) continue;
    if (pwm50Motor == i) pwm50Motor = -1;
    stopMotor(i, "safety timeout");
    motorCoast(i);
    Serial.print("M");
    Serial.print(i + 1);
    Serial.println(" safety timeout — coasted");
  }
}

// ---------- Fault ----------
bool faultAsserted() {
  return digitalRead(FAULT_PIN) == LOW;  // active-low nFAULT
}

void handleFault() {
  if (!faultAsserted()) return;
  if (faultLatched) return;
  faultLatched = true;
  pwm50Motor = -1;
  stopAllMotors("fault");
  motorDisableAll();
  Serial.println("!!! DRIVER FAULT (GPIO47) — all EN low. 'fault clear' to reset latch");
  if (deviceConnected && pTxCharacteristic != nullptr) {
    pTxCharacteristic->setValue("FAULT\n");
    pTxCharacteristic->notify();
  }
}

void enableDriversStaggered() {
  Serial.println("Enabling drivers staggered...");
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (!motorReady[i]) continue;
    if (faultAsserted()) {
      handleFault();
      Serial.println("FAULT during stagger — abort enable");
      return;
    }
    motorEnable(i);
    Serial.print("  enabled M");
    Serial.print(i + 1);
    Serial.print(" (");
    Serial.print(DRIVER_REF[i]);
    Serial.println(")");
    delay(DRIVER_ENABLE_STAGGER_MS);
  }
  Serial.println("Driver enable done");
}

// ---------- 50% duty test ----------
void applyPwm50Duty(int motorIndex) {
  if (motorIndex < 0 || motorIndex >= NUM_MOTORS) return;
  if (!motorReady[motorIndex] || !driverUsable(motorIndex)) return;
  markMotorEnergised(motorIndex);
  float half = DRIVER_BUS_VOLTAGE * 0.5f;
  driverFor[motorIndex]->setPwm(half, half, half);
}

void stopPwm50() {
  if (pwm50Motor < 0) return;
  int m = pwm50Motor;
  pwm50Motor = -1;
  motorCoast(m);
  Serial.print("pwm50 M");
  Serial.print(m + 1);
  Serial.println(" stopped (coast)");
}

void startPwm50(int motorIndex) {
  if (!motorReady[motorIndex]) {
    Serial.println("motor not ready");
    return;
  }
  if (faultLatched) {
    Serial.println("FAULT latched — clear first");
    return;
  }
  stopMotor(motorIndex, "pwm50");
  if (pwm50Motor >= 0 && pwm50Motor != motorIndex) stopPwm50();

  motorEnable(motorIndex);
  pwm50Motor = motorIndex;
  applyPwm50Duty(motorIndex);
  Serial.print("pwm50 M");
  Serial.print(motorIndex + 1);
  Serial.print(" (");
  Serial.print(DRIVER_REF[motorIndex]);
  Serial.println(") — all phases 50%. Send 'pwm50 off' to stop");
}

// ---------- Calibration ----------
bool setEnd(int i, bool asMax) {
  if (!sensorOk[i] && !initOneSensor(i)) {
    Serial.print("M");
    Serial.print(i + 1);
    Serial.println(" no sensor");
    return false;
  }
  updateContinuousAngle(i);
  float ang = motorAngle(i);
  if (asMax) {
    angleMax[i] = ang;
    haveMax[i] = true;
  } else {
    angleMin[i] = ang;
    haveMin[i] = true;
  }
  refreshCalibrated(i);

  Serial.print("M");
  Serial.print(i + 1);
  Serial.print(asMax ? " MAX=" : " MIN=");
  Serial.print(ang * 180.0f / PI, 1);
  Serial.print("deg");
  if (calibrated[i]) {
    Serial.print("  OK span=");
    Serial.print((angleMax[i] - angleMin[i]) * 180.0f / PI, 1);
    Serial.println("deg");
  } else if (haveMin[i] && haveMax[i]) {
    Serial.println("  FAIL span too small");
  } else {
    Serial.println(asMax ? "  (still need min)" : "  (still need max)");
  }
  return true;
}

void setEndAll(bool asMax) {
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (sensorOk[i] || initOneSensor(i)) setEnd(i, asMax);
  }
}

// ---------- Motion commands (non-blocking) ----------
void traceStart(int i) {
  traceActive[i] = true;
  nextTraceMs[i] = millis() + TRACE_MS;
  traceStartAngle[i] = motorAngle(i);
}

void traceMotor(int i) {
  if (!traceActive[i]) return;
  if ((long)(millis() - nextTraceMs[i]) < 0) return;
  nextTraceMs[i] = millis() + TRACE_MS;

  Serial.print("  M");
  Serial.print(i + 1);
  Serial.print(" ang=");
  Serial.print(motorAngle(i) * 180.0f / PI, 1);
  Serial.print(" d=");
  Serial.print((motorAngle(i) - traceStartAngle[i]) * 180.0f / PI, 1);
  Serial.print(" vel=");
  Serial.print(commandSign(i) * motors[i].shaft_velocity, 2);
  if (focDebug) {
    Serial.print(" Uq=");
    Serial.print(motors[i].voltage.q, 2);
    Serial.print(" Iq=");
    Serial.print(motors[i].current.q, 3);
  }
  Serial.println();
}

bool prepareMotion(int i) {
  if (i < 0 || i >= NUM_MOTORS) return false;
  if (!motorReady[i]) {
    Serial.print("M");
    Serial.print(i + 1);
    Serial.println(" not ready — skip");
    return false;
  }
  if (faultLatched) {
    Serial.println("FAULT latched — skip motion");
    return false;
  }
  if (pwm50Motor == i) stopPwm50();
  return true;
}

void startVelocity(int i, float logicalVel, unsigned long durationMs) {
  if (!prepareMotion(i)) return;

  if (calibrated[i] && sensorOk[i]) {
    updateContinuousAngle(i);
    float pos = normalizedPosition(i);
    if (!isnan(pos)) {
      if (logicalVel > 0 && pos > 0.95f) {
        Serial.println("  at max end — skip");
        return;
      }
      if (logicalVel < 0 && pos < 0.05f) {
        Serial.println("  at min end — skip");
        return;
      }
    }
  }

  motors[i].controller = focReady[i] ? MotionControlType::velocity
                                     : MotionControlType::velocity_openloop;
  if (!focReady[i]) motors[i].voltage_limit = OPENLOOP_VOLTAGE_LIMIT;
  motors[i].PID_velocity.reset();

  ctrlMode[i] = CTRL_VELOCITY;
  ctrlTarget[i] = logicalVel;
  ctrlEndMs[i] = millis() + durationMs;
  ctrlHold[i] = false;

  motorEnable(i);
  traceStart(i);

  Serial.print("jog M");
  Serial.print(i + 1);
  Serial.print(focReady[i] ? " closed " : " open ");
  Serial.print(logicalVel, 2);
  Serial.print("rad/s start=");
  Serial.println(motorAngle(i) * 180.0f / PI, 1);
}

void startPosition(int i, float logicalAngle, bool hold) {
  if (!prepareMotion(i)) return;
  if (!focReady[i]) {
    Serial.print("M");
    Serial.print(i + 1);
    Serial.println(" needs FOC for position control");
    return;
  }
  if (calibrated[i]) {
    logicalAngle = _constrain(logicalAngle, angleMin[i], angleMax[i]);
  }
  updateContinuousAngle(i);

  motors[i].controller = MotionControlType::angle;
  motors[i].PID_velocity.reset();
  motors[i].P_angle.reset();

  ctrlMode[i] = CTRL_POSITION;
  ctrlTarget[i] = logicalAngle;
  ctrlHold[i] = hold;
  ctrlEndMs[i] = hold ? 0 : millis() + POSITION_TIMEOUT_MS;
  posInToleranceMs[i] = 0;

  motorEnable(i);
  traceStart(i);

  Serial.print(hold ? "hold M" : "goto M");
  Serial.print(i + 1);
  Serial.print(" target=");
  Serial.print(logicalAngle * 180.0f / PI, 1);
  Serial.print("deg from=");
  Serial.println(motorAngle(i) * 180.0f / PI, 1);
}

void startTorque(int i, float amps) {
  if (!prepareMotion(i)) return;
  if (!focReady[i]) {
    Serial.print("M");
    Serial.print(i + 1);
    Serial.println(" needs FOC for torque control");
    return;
  }
  amps = _constrain(amps, -MOTOR_CURRENT_LIMIT, MOTOR_CURRENT_LIMIT);

  motors[i].controller = MotionControlType::torque;
  motors[i].PID_velocity.reset();

  ctrlMode[i] = CTRL_TORQUE;
  ctrlTarget[i] = amps;
  ctrlEndMs[i] = millis() + TORQUE_TIMEOUT_MS;
  ctrlHold[i] = false;

  motorEnable(i);
  traceStart(i);

  Serial.print("torque M");
  Serial.print(i + 1);
  Serial.print(" = ");
  Serial.print(amps, 3);
  Serial.println("A");
}

// Runs one control iteration for every active motor. Called from loop().
void serviceMotors() {
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (ctrlMode[i] == CTRL_IDLE) continue;

    if (!motorReady[i] || faultLatched) {
      stopMotor(i, "unavailable");
      continue;
    }
    if (ctrlEndMs[i] != 0 && (long)(millis() - ctrlEndMs[i]) >= 0) {
      stopMotor(i, "timeout");
      continue;
    }

    if (focReady[i]) {
      motors[i].loopFOC();  // also refreshes the sensor
      if (sensorOk[i]) continuousAngle[i] = focSensors[i].getAngle();
    } else if (sensorOk[i]) {
      // open-loop fallback: loopFOC() returns early, so read the encoder ourselves
      updateContinuousAngle(i);
    }

    float sign = commandSign(i);
    float shaftTarget = 0;
    bool reachedEnd = false;

    switch (ctrlMode[i]) {
      case CTRL_VELOCITY: {
        float pos = normalizedPosition(i);
        if (!isnan(pos)) {
          if (ctrlTarget[i] > 0 && pos >= 0.98f) reachedEnd = true;
          if (ctrlTarget[i] < 0 && pos <= 0.02f) reachedEnd = true;
        }
        shaftTarget = sign * ctrlTarget[i];
        break;
      }
      case CTRL_POSITION: {
        float err = ctrlTarget[i] - motorAngle(i);
        if (fabsf(err) <= POSITION_TOLERANCE_RAD) {
          if (posInToleranceMs[i] == 0) {
            posInToleranceMs[i] = millis();
          } else if (!ctrlHold[i] && millis() - posInToleranceMs[i] >= POSITION_SETTLE_MS) {
            reachedEnd = true;
          }
        } else {
          posInToleranceMs[i] = 0;
        }
        shaftTarget = sign * ctrlTarget[i];
        break;
      }
      case CTRL_TORQUE:
        shaftTarget = sign * ctrlTarget[i];
        break;
      default:
        break;
    }

    if (reachedEnd) {
      stopMotor(i, ctrlMode[i] == CTRL_POSITION ? "reached" : "endstop");
      continue;
    }

    markMotorEnergised(i);
    motors[i].move(shaftTarget);
    traceMotor(i);
  }
  checkMotorEnergiseTimeout();
}

void startDemo() {
  demoIndex = 0;
  demoStepStarted = false;
}

// For each motor in turn: goto min, then goto max. Skips motors that are not
// ready, FOC-aligned, or min/max calibrated.
void serviceDemo() {
  if (demoIndex < 0) return;

  while (true) {
    int m = demoIndex / 2;
    if (m >= NUM_MOTORS) {
      demoIndex = -1;
      Serial.println("demo done");
      return;
    }

    if (!motorReady[m] || !focReady[m] || !calibrated[m]) {
      Serial.print("demo skip M");
      Serial.print(m + 1);
      if (!motorReady[m]) Serial.println(" (not ready)");
      else if (!focReady[m]) Serial.println(" (needs FOC)");
      else Serial.println(" (not calibrated — set min and max)");
      demoIndex = (m + 1) * 2;
      demoStepStarted = false;
      continue;
    }

    if (!demoStepStarted) {
      bool toMax = (demoIndex % 2) == 1;
      float target = toMax ? angleMax[m] : angleMin[m];
      Serial.print("demo M");
      Serial.print(m + 1);
      Serial.println(toMax ? " -> max" : " -> min");
      startPosition(m, target, false);
      demoStepStarted = true;
      return;
    }

    if (ctrlMode[m] == CTRL_IDLE) {
      demoIndex++;
      demoStepStarted = false;
      continue;
    }
    return;
  }
}

// ---------- Init ----------
void applyClosedLoopTuning(int i) {
  motors[i].voltage_limit = CLOSED_LOOP_VOLTAGE_LIMIT;
  motors[i].updateTorqueControlType(TorqueControlType::estimated_current);
  motors[i].updateCurrentLimit(MOTOR_CURRENT_LIMIT);
  motors[i].updateVelocityLimit(MAX_VELOCITY);
  motors[i].PID_velocity.P = VEL_P;
  motors[i].PID_velocity.I = VEL_I;
  motors[i].PID_velocity.D = VEL_D;
  motors[i].LPF_velocity.Tf = VEL_LPF_TF;
  motors[i].P_angle.P = ANGLE_P;
  motors[i].LPF_angle.Tf = 0.0f;
}

bool runFocAlignment(int i) {
  motorEnable(i);
  delay(50);
  bool ok = motors[i].initFOC() != 0;
  focReady[i] = ok;
  if (ok) {
    applyClosedLoopTuning(i);
    Serial.print("  M");
    Serial.print(i + 1);
    Serial.print(" initFOC OK  dir=");
    Serial.print((int)motors[i].sensor_direction);
    Serial.print(" zero=");
    Serial.print(motors[i].zero_electric_angle, 3);
    Serial.print(" pp_check=");
    Serial.println(motors[i].pp_check_result ? "ok" : "FAIL");
  } else {
    Serial.print("  M");
    Serial.print(i + 1);
    Serial.println(" initFOC FAIL — open-loop fallback");
    motors[i].controller = MotionControlType::velocity_openloop;
    motors[i].updateTorqueControlType(TorqueControlType::voltage);
    motors[i].voltage_limit = OPENLOOP_VOLTAGE_LIMIT;
  }
  motorCoast(i);
  motorDisable(i);
  return ok;
}

bool initMotor(int i) {
  BLDCDriver *drv = driverFor[i];
  drv->voltage_power_supply = DRIVER_BUS_VOLTAGE;
  drv->voltage_limit = DRIVER_BUS_VOLTAGE;
  if (!drv->init()) return false;

  motors[i].linkDriver(drv);
  if (sensorOk[i]) motors[i].linkSensor(&focSensors[i]);
  motors[i].controller = MotionControlType::velocity;
  motors[i].torque_controller = TorqueControlType::voltage;  // alignment runs on voltage
  motors[i].voltage_limit = CLOSED_LOOP_VOLTAGE_LIMIT;
  motors[i].current_limit = MOTOR_CURRENT_LIMIT;
  motors[i].velocity_limit = MAX_VELOCITY;
  motors[i].voltage_sensor_align = ALIGN_VOLTAGE;

  if (!motors[i].init()) return false;

  focReady[i] = false;
  if (sensorOk[i]) {
    runFocAlignment(i);
  } else {
    motors[i].controller = MotionControlType::velocity_openloop;
    motors[i].voltage_limit = OPENLOOP_VOLTAGE_LIMIT;
    motorCoast(i);
    motorDisable(i);
  }
  return true;
}

void disableAllEnPinsEarly() {
  for (int i = 0; i < NUM_MOTORS; i++) {
    pinMode(EN_PINS[i], OUTPUT);
    digitalWrite(EN_PINS[i], LOW);
  }
}

// ---------- Telemetry ----------
void streamTelemetry() {
  unsigned long ts = millis();

  String motorsLine = "values motors ";
  String anglesLine = "values angles ";
  for (int i = 0; i < NUM_MOTORS; i++) {
    // Motors under active control already refreshed their angle in serviceMotors()
    if (ctrlMode[i] == CTRL_IDLE) {
      if (!selectMuxChannel(MUX_CHANNELS[i]) || !as5600Present()) {
        sensorOk[i] = false;
      } else {
        sensorOk[i] = true;
        updateContinuousAngle(i);
      }
    }
    if (!sensorOk[i]) {
      motorsLine += "nan,";
      anglesLine += "nan,";
      continue;
    }

    anglesLine += String(motorAngle(i) * 180.0f / PI, 1);
    anglesLine += ",";

    float pos = normalizedPosition(i);
    if (isnan(pos)) {
      motorsLine += "nan,";
    } else {
      motorsLine += String(pos, 3);
      motorsLine += ",";
    }
  }
  motorsLine += "\n";
  anglesLine += "\n";

  sendValues(addTimestampToValues(motorsLine, ts));
  // Brief gap so BLE notify can flush (ah-rc style clients expect separate lines)
  delay(2);
  sendValues(addTimestampToValues(anglesLine, ts));
}

const char *modeName(CtrlMode m) {
  switch (m) {
    case CTRL_VELOCITY: return "velocity";
    case CTRL_POSITION: return "position";
    case CTRL_TORQUE:   return "torque";
    default:            return "idle";
  }
}

void printStatus() {
  for (int i = 0; i < NUM_MOTORS; i++) {
    Serial.print("M");
    Serial.print(i + 1);
    Serial.print(" ");
    Serial.print(DRIVER_REF[i]);
    Serial.print(i == LEDC_MOTOR_INDEX ? " [LEDC]" : "      ");
    Serial.print(" ready=");
    Serial.print(motorReady[i] ? "y" : "n");
    Serial.print(" foc=");
    Serial.print(focReady[i] ? "y" : "n");
    Serial.print(" dir=");
    Serial.print((int)motors[i].sensor_direction);
    Serial.print(" cal=");
    Serial.print(calibrated[i] ? "y" : "n");
    Serial.print(" mode=");
    Serial.print(modeName(ctrlMode[i]));
    Serial.print(" ang=");
    Serial.print(motorAngle(i) * 180.0f / PI, 1);
    float pos = normalizedPosition(i);
    if (!isnan(pos)) {
      Serial.print(" pos=");
      Serial.print(pos, 3);
    }
    Serial.println();
  }
}

void printHelp() {
  Serial.println("Commands:");
  Serial.println("  min | max          mark current pose as min/max (all motors)");
  Serial.println("  min N | max N      same for motor N (1..5)");
  Serial.println("  fwd N [s] | rev N [s]   velocity jog (default 2s)");
  Serial.println("  goto N <0..1>      move to calibrated position");
  Serial.println("  hold N             hold current angle indefinitely");
  Serial.println("  torque N <amps>    torque command (+/-)");
  Serial.println("  idle N | idle      stop motion, coast");
  Serial.println("  en N | dis N       enable / disable driver (EN pin)");
  Serial.println("  pwm50 N | pwm50 off  hold 50% duty on all phases");
  Serial.println("  fault | fault clear");
  Serial.println("  realign N          re-run FOC alignment for motor N");
  Serial.println("  status | focdebug on|off");
  Serial.println("  start | stop       enable/disable values stream");
  Serial.println("  get version");
  Serial.println("  sensors | muxscan | help");
  Serial.println("  demo               each calibrated motor: goto min, then max");
  Serial.println("  imu reset          reset Madgwick orientation filter");
  Serial.println("  (values quat / values linaccel / values flex stream when start is active)");
}

// ---------- Command parsing ----------
int motorArg(const String &cmd, int fromIndex) {
  return cmd.substring(fromIndex).toInt();
}

void handleCommand(String cmd) {
  cmd.trim();
  cmd.toLowerCase();
  if (cmd.length() == 0) return;

  Serial.print(">: ");
  Serial.println(cmd);

  if (cmd == "min") {
    setEndAll(false);
  } else if (cmd == "max") {
    setEndAll(true);
  } else if (cmd.startsWith("min ")) {
    int n = motorArg(cmd, 4);
    if (n >= 1 && n <= NUM_MOTORS) setEnd(n - 1, false);
    else Serial.println("Usage: min N");
  } else if (cmd.startsWith("max ")) {
    int n = motorArg(cmd, 4);
    if (n >= 1 && n <= NUM_MOTORS) setEnd(n - 1, true);
    else Serial.println("Usage: max N");
  } else if (cmd.startsWith("fwd ") || cmd.startsWith("rev ")) {
    bool forward = cmd.startsWith("fwd ");
    String args = cmd.substring(4);
    args.trim();
    int sp = args.indexOf(' ');
    int n = (sp < 0 ? args : args.substring(0, sp)).toInt();
    float secs = (sp < 0) ? 0.0f : args.substring(sp + 1).toFloat();
    unsigned long durMs = (secs > 0.05f) ? (unsigned long)(secs * 1000.0f) : DEMO_MOVE_MS;
    if (n >= 1 && n <= NUM_MOTORS) {
      startVelocity(n - 1, forward ? +DEMO_SPEED : -DEMO_SPEED, durMs);
    } else {
      Serial.println("Usage: fwd N [seconds]");
    }
  } else if (cmd.startsWith("goto ")) {
    String args = cmd.substring(5);
    args.trim();
    int sp = args.indexOf(' ');
    if (sp < 0) {
      Serial.println("Usage: goto N <0..1>");
    } else {
      int n = args.substring(0, sp).toInt();
      float pos = args.substring(sp + 1).toFloat();
      if (n < 1 || n > NUM_MOTORS) {
        Serial.println("Usage: goto N <0..1>");
      } else if (!calibrated[n - 1]) {
        Serial.println("motor not calibrated — set min and max first");
      } else {
        int i = n - 1;
        pos = _constrain(pos, 0.0f, 1.0f);
        startPosition(i, angleMin[i] + pos * (angleMax[i] - angleMin[i]), false);
      }
    }
  } else if (cmd.startsWith("hold ")) {
    int n = motorArg(cmd, 5);
    if (n >= 1 && n <= NUM_MOTORS) {
      updateContinuousAngle(n - 1);
      startPosition(n - 1, motorAngle(n - 1), true);
    } else Serial.println("Usage: hold N");
  } else if (cmd.startsWith("torque ")) {
    String args = cmd.substring(7);
    args.trim();
    int sp = args.indexOf(' ');
    if (sp < 0) {
      Serial.println("Usage: torque N <amps>");
    } else {
      int n = args.substring(0, sp).toInt();
      float amps = args.substring(sp + 1).toFloat();
      if (n >= 1 && n <= NUM_MOTORS) startTorque(n - 1, amps);
      else Serial.println("Usage: torque N <amps>");
    }
  } else if (cmd == "idle") {
    demoIndex = -1;
    stopAllMotors("user");
    Serial.println("all motors idle");
  } else if (cmd.startsWith("idle ")) {
    int n = motorArg(cmd, 5);
    if (n >= 1 && n <= NUM_MOTORS) {
      stopMotor(n - 1, "user");
      Serial.print("M");
      Serial.print(n);
      Serial.println(" idle");
    } else Serial.println("Usage: idle N");
  } else if (cmd.startsWith("dis ")) {
    int n = motorArg(cmd, 4);
    if (n >= 1 && n <= NUM_MOTORS) {
      if (pwm50Motor == n - 1) pwm50Motor = -1;
      stopMotor(n - 1, "disable");
      motorDisable(n - 1);
      Serial.print("M");
      Serial.print(n);
      Serial.println(" disabled (EN low)");
    } else Serial.println("Usage: dis N");
  } else if (cmd == "pwm50 off" || cmd == "pwm50off") {
    stopPwm50();
  } else if (cmd.startsWith("pwm50 ")) {
    int n = motorArg(cmd, 6);
    if (n >= 1 && n <= NUM_MOTORS) startPwm50(n - 1);
    else Serial.println("Usage: pwm50 N  or  pwm50 off");
  } else if (cmd.startsWith("en ")) {
    int n = motorArg(cmd, 3);
    if (n >= 1 && n <= NUM_MOTORS) {
      motorEnable(n - 1);
      Serial.print("M");
      Serial.print(n);
      Serial.println(" enabled (EN high, coasting)");
    } else Serial.println("Usage: en N");
  } else if (cmd == "fault") {
    Serial.print("FAULT pin=");
    Serial.print(faultAsserted() ? "ACTIVE" : "ok");
    Serial.print(" latched=");
    Serial.println(faultLatched ? "yes" : "no");
  } else if (cmd == "fault clear") {
    if (faultAsserted()) {
      Serial.println("FAULT still active on GPIO47 — fix hardware first");
    } else {
      faultLatched = false;
      Serial.println("FAULT latch cleared");
    }
  } else if (cmd.startsWith("realign ")) {
    int n = motorArg(cmd, 8);
    if (n < 1 || n > NUM_MOTORS) {
      Serial.println("Usage: realign N");
    } else if (!sensorOk[n - 1] || !motorReady[n - 1]) {
      Serial.println("motor/sensor not ready");
    } else if (faultLatched) {
      Serial.println("FAULT latched — clear first");
    } else {
      int i = n - 1;
      stopMotor(i, "realign");
      motors[i].sensor_direction = Direction::UNKNOWN;
      motors[i].zero_electric_angle = NOT_SET;
      motors[i].updateTorqueControlType(TorqueControlType::voltage);
      motors[i].voltage_limit = CLOSED_LOOP_VOLTAGE_LIMIT;
      runFocAlignment(i);
      motorEnable(i);
    }
  } else if (cmd == "imu reset") {
    imuFilterReset();
    Serial.println("IMU filter reset");
  } else if (cmd == "status") {
    printStatus();
  } else if (cmd == "focdebug on") {
    focDebug = true;
    Serial.println("FOC debug on");
  } else if (cmd == "focdebug off") {
    focDebug = false;
    Serial.println("FOC debug off");
  } else if (cmd == "start" || cmd == "debug off") {
    txValues = true;
    Serial.println("Sensor transmission started");
  } else if (cmd == "stop" || cmd == "debug on") {
    txValues = false;
    Serial.println("Sensor transmission stopped");
  } else if (cmd == "get version" || cmd.startsWith("get version")) {
    Serial.print("version:");
    Serial.println(FIRMWARE_VERSION);
    if (deviceConnected && pTxCharacteristic != nullptr) {
      String v = String("version:") + FIRMWARE_VERSION + "\n";
      pTxCharacteristic->setValue(v.c_str());
      pTxCharacteristic->notify();
    }
  } else if (cmd == "sensors") {
    printAllSensors();
  } else if (cmd == "muxscan") {
    muxScan();
  } else if (cmd == "demo") {
    startDemo();
    Serial.println("demo started — each motor min then max (idle to cancel)");
  } else if (cmd == "help") {
    printHelp();
  } else {
    Serial.print("Unknown: ");
    Serial.println(cmd);
    printHelp();
  }
}

// Shared by USB Serial and BLE RX (mBle.h)
void parse(byte buffer[], int l) {
  String cmd = "";
  for (int n = 0; n < l; n++) cmd += (char)buffer[n];
  handleCommand(cmd);
}

void setup() {
  disableAllEnPinsEarly();

  Serial.begin(115200);
  Serial.setTimeout(10);
  comRxInit();
  delay(1000);
  Serial.println("OH1-CORE v1.0 — 5-motor GMT4108");
  Serial.print("version:");
  Serial.println(FIRMWARE_VERSION);

  char BLName[] = "OMNIHUMAN-O1";
  startBle(BLName);
  Serial.print("BLE name: ");
  Serial.println(BLName);

  pinMode(FAULT_PIN, INPUT_PULLUP);
  Serial.print("FAULT bus GPIO");
  Serial.print(FAULT_PIN);
  Serial.println(faultAsserted() ? " ACTIVE at boot" : " ok");

  beginHostI2C();
  delay(50);
  if (probeMux()) {
    Serial.print("TCA9548A OK at 0x");
    Serial.println(muxAddr, HEX);
  } else {
    Serial.println("TCA9548A NOT found on first probe — still trying sensors");
  }

  for (int i = 0; i < NUM_MOTORS; i++) {
    sensorOk[i] = initOneSensor(i);
    Serial.print("Sensor ");
    Serial.print(i + 1);
    if (sensorOk[i]) {
      Serial.print(" OK ");
      Serial.println(motorAngle(i) * 180.0f / PI, 1);
    } else {
      Serial.println(" FAIL (unplugged or no ACK)");
    }
  }
  beginHostI2C();

  // Each motor: driver init, then FOC alignment with only its own EN high
  for (int i = 0; i < NUM_MOTORS; i++) {
    Serial.print("Motor ");
    Serial.print(i + 1);
    Serial.print(" (");
    Serial.print(DRIVER_REF[i]);
    Serial.println(i == LEDC_MOTOR_INDEX ? ") LEDC init..." : ") init...");
    motorReady[i] = initMotor(i);
    if (!motorReady[i]) {
      Serial.print("  M");
      Serial.print(i + 1);
      Serial.println(" driver/motor init FAILED");
    }
  }

  // Stagger EN high so inrush / bias doesn't hit all at once
  enableDriversStaggered();

  beginHostI2C();
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (!sensorOk[i]) sensorOk[i] = initOneSensor(i);
  }
  printAllSensors();
  printStatus();
  printHelp();
  Serial.println("Move each rail to an end, then send min or max");
  Serial.println("Streaming values motors / values angles (start|stop to gate)");
  Serial.println("RS485 glove on GPIO44 RX — values quat / values linaccel / values flex when streaming");
}

void loop() {
  handleFault();
  comRxPoll();

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (inputBuffer.length() > 0) {
        handleCommand(inputBuffer);
        inputBuffer = "";
      }
    } else {
      inputBuffer += c;
    }
  }

  serviceMotors();
  serviceDemo();

  // Keep refreshing 50% duty while test mode is active
  static unsigned long nextPwm50Refresh = 0;
  if (pwm50Motor >= 0 && millis() >= nextPwm50Refresh) {
    nextPwm50Refresh = millis() + 50;
    applyPwm50Duty(pwm50Motor);
  }

  static unsigned long nextStream = 0;
  if (txValues && millis() >= nextStream) {
    nextStream = millis() + STREAM_MS;
    streamTelemetry();
  }
}
