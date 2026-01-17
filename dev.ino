#include <ESP32Servo.h>
#include <TinyGPS++.h>
#include <EEPROM.h>
#include "BLEDevice.h"
#include "BLEServer.h"
#include "BLEUtils.h"
#include "BLE2902.h"


const char* MODEL_NAME = "AEROTEQ Nano";
const char* FIRMWARE_VER = "v2.0-Prod";
// ตั้งค่า ID ของรุ่น(aero-001, aero-002)
const char* SERIAL_SERIES = "aero-001";


#define BLE_SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define BLE_CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"


const int SERVO1_PIN = 13;
const int SERVO2_PIN = 12;
const int BUTTON_PIN = 10;
const int GPS_RX_PIN = 16;
const int GPS_TX_PIN = 17;

const unsigned long SERVO_MOVE_TIME = 1000;
const unsigned long STATIONARY_TIME = 120000;
const unsigned long SPEED_TIMEOUT   = 5000;
const unsigned long DEBOUNCE_DELAY  = 50;
const int EEPROM_SIZE = 512;


TinyGPSPlus gps;
HardwareSerial GPS(2);
Servo s1, s2;


bool isServoMoving = false;
unsigned long servoMoveStart = 0;

int buttonLogicState = HIGH;
int lastButtonPhysState = HIGH;
unsigned long lastDebounceTime = 0;

int angles[7] = {0, 40, 80, 110, 140, 160, 180}; // Index 1..6
int currentAngle = 0;
int lastPos = 6;
bool attached = false;

float vHigh = 50, vLow = 50;
float curSpeed = 0;
bool singleThreshEnabled = false;
float vThresh = 50;
float hyst = 0;

bool autoMode = true;
unsigned long tLastMove = 0, tLastSpeed = 0;

bool invertS1 = false;
bool invertS2 = true;

volatile bool bleCommandFlag = false;
volatile char bleCommandToProcess[128];
BLEServer* pServer = nullptr;
BLECharacteristic* pChar = nullptr;
bool bleConnected = false;
String fullDeviceName; // เก็บชื่อเต็มพร้อม MAC Address

const int ADDR_LAST_POS = 24;
const int ADDR_HIGH     = 28;
const int ADDR_LOW      = 32;
const int ADDR_VTHRESH  = 36;
const int ADDR_STEN     = 40;
const int ADDR_HYST     = 44;
const int ADDR_INV_S1   = 48;
const int ADDR_INV_S2   = 52;

// ================= HELPERS =================
int clampAngle(int a, int def) { return (a < 0 || a > 180) ? def : a; }


int getRealAngle1(int target) { return invertS1 ? 180 - target : target; }
int getRealAngle2(int target) { return invertS2 ? 180 - target : target; }

void attachServos() {
  if (!attached) {

    s1.write(getRealAngle1(currentAngle));
    s2.write(getRealAngle2(currentAngle));

    s1.attach(SERVO1_PIN);
    s2.attach(SERVO2_PIN);
    attached = true;
  }
}

void detachServos() {
  if (attached) {
    s1.detach();
    s2.detach();
    attached = false;
  }
}

void startMoveTo(int target) {
  // ถ้าองศาเดิมและยัง Attach อยู่ ไม่ต้องทำอะไร
  if (target == currentAngle && attached) return;

  currentAngle = target;

  // Attach (พร้อม Anti-Jitter ภายในฟังก์ชัน)
  attachServos();

  // ย้ำค่าอีกครั้งเพื่อความชัวร์
  s1.write(getRealAngle1(target));
  s2.write(getRealAngle2(target));

  isServoMoving = true;
  servoMoveStart = millis();
}

void servoTick() {
  if (isServoMoving) {
    if (millis() - servoMoveStart > SERVO_MOVE_TIME) {
      detachServos();
      isServoMoving = false;
    }
  }
}

void executeMove(int posIndex) {
  if (posIndex < 1 || posIndex > 6) return;


  if (lastPos != posIndex) {
    lastPos = posIndex;
    EEPROM.writeInt(ADDR_LAST_POS, lastPos);
    EEPROM.commit();
  }

  Serial.printf("[Action] Moving to M%d (%d deg)\n", lastPos, angles[lastPos]);
  startMoveTo(angles[lastPos]);
}

void saveSettings() {
  EEPROM.writeFloat(ADDR_HIGH, vHigh);
  EEPROM.writeFloat(ADDR_LOW, vLow);
  EEPROM.writeFloat(ADDR_VTHRESH, vThresh);
  EEPROM.writeInt(ADDR_STEN, singleThreshEnabled ? 1 : 0);
  EEPROM.writeFloat(ADDR_HYST, hyst);
  EEPROM.writeInt(ADDR_INV_S1, invertS1 ? 1 : 0);
  EEPROM.writeInt(ADDR_INV_S2, invertS2 ? 1 : 0);
  EEPROM.commit();
}

void loadEEP() {
  if (!EEPROM.begin(EEPROM_SIZE)) return;
  for (int i = 1; i <= 6; i++) {
    int v = EEPROM.readInt(((i - 1) * 4));
    if (v != 0 || i == 6) angles[i] = clampAngle(v, angles[i]);
  }
  lastPos = EEPROM.readInt(ADDR_LAST_POS);
  if (lastPos < 1 || lastPos > 6) lastPos = 6;

  float vh = EEPROM.readFloat(ADDR_HIGH);
  if (vh > 0) vHigh = vh;
  float vl = EEPROM.readFloat(ADDR_LOW);
  if (vl > 0) vLow = vl;

  vThresh = EEPROM.readFloat(ADDR_VTHRESH);
  if (vThresh < 0) vThresh = 50;
  singleThreshEnabled = (EEPROM.readInt(ADDR_STEN) == 1);
  hyst = EEPROM.readFloat(ADDR_HYST);
  if(hyst < 0) hyst = 0;

  int inv1 = EEPROM.readInt(ADDR_INV_S1);
  int inv2 = EEPROM.readInt(ADDR_INV_S2);
  if (inv1 != -1) invertS1 = (inv1 == 1);
  if (inv2 != -1) invertS2 = (inv2 == 1);
}

// ================= BLE SETUP =================
class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) { bleConnected = true; }
  void onDisconnect(BLEServer* pServer) {
    bleConnected = false;
    pServer->getAdvertising()->start();
  }
};

class MyCharCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* ch) {
    String rx = ch->getValue();
    if (rx.length() > 0 && rx.length() < sizeof(bleCommandToProcess)) {
      strncpy((char*)bleCommandToProcess, rx.c_str(), sizeof(bleCommandToProcess) - 1);
      bleCommandToProcess[sizeof(bleCommandToProcess) - 1] = '\0';
      bleCommandFlag = true;
    }
  }
};

void bleInit() {
  // สร้าง Unique Name โดยดึง MAC Address 2 ตัวท้ายมาต่อท้ายชื่อ
  // ผลลัพธ์จะเป็น: "AEROTEQ Nano A1B2" (ไม่ซ้ำกันแต่ละบอร์ด)
  uint64_t chipid = ESP.getEfuseMac();
  uint16_t chip = (uint16_t)(chipid >> 32);
  char suffix[8];
  snprintf(suffix, sizeof(suffix), " %04X", chip);
  fullDeviceName = String(MODEL_NAME) + String(suffix);

  BLEDevice::init(fullDeviceName.c_str());
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(BLE_SERVICE_UUID);
  pChar = pService->createCharacteristic(BLE_CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);
  pChar->addDescriptor(new BLE2902());
  pChar->setCallbacks(new MyCharCallbacks());
  pService->start();

  // *** CRITICAL FOR APP FILTERING ***
  // บังคับ Advertise Service UUID เพื่อให้ App กรองหาเจอได้ง่าย
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(BLE_SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();

  Serial.println("BLE Started as: " + fullDeviceName);
}

void bleNotify(String msg) {
  if (bleConnected && pChar != nullptr) {
    pChar->setValue((uint8_t*)msg.c_str(), msg.length());
    pChar->notify();
  }
}

// ================= LOGIC & COMMANDS =================
void handleCmd(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;
  Serial.println("> CMD: " + cmd);

  if (toupper(cmd[0]) == 'M') {
    int pos = cmd.substring(1).toInt();
    if (pos >= 1 && pos <= 6) { executeMove(pos); autoMode = false; }
  }
  else if (cmd.equalsIgnoreCase("OPEN")) { executeMove(1); autoMode = false; }
  else if (cmd.equalsIgnoreCase("CLOSE")) { executeMove(6); autoMode = false; }
  else if (cmd.equalsIgnoreCase("AUTO")) { autoMode = true; }

  // --- SETTINGS ---
  else if (cmd.startsWith("SET HIGH ")) { vHigh = cmd.substring(9).toFloat(); saveSettings(); }
  else if (cmd.startsWith("SET LOW ")) { vLow = cmd.substring(8).toFloat(); saveSettings(); }
  else if (cmd.startsWith("SET HYST ")) { hyst = cmd.substring(9).toFloat(); saveSettings(); }
  else if (cmd.equalsIgnoreCase("INVERT S1")) { invertS1 = !invertS1; saveSettings(); }
  else if (cmd.equalsIgnoreCase("INVERT S2")) { invertS2 = !invertS2; saveSettings(); }

  // --- IDENTITY COMMAND ---
  // App สามารถส่งคำสั่ง "WHO" มาถามได้ว่านี่คือบอร์ดรุ่นไหน ID อะไร
  else if (cmd.equalsIgnoreCase("WHO") || cmd.equalsIgnoreCase("INFO")) {
      char info[64];
      snprintf(info, sizeof(info), "ID|%s|%s|%s", MODEL_NAME, SERIAL_SERIES, FIRMWARE_VER);
      bleNotify(String(info));
  }

  else if (cmd.equalsIgnoreCase("REBOOT")) {
    delay(200); ESP.restart();
  }
  else if (cmd.equalsIgnoreCase("RESET FACTORY")) {
    for (int i = 0; i < EEPROM_SIZE; i++) EEPROM.write(i, 0);
    EEPROM.commit();
    delay(200); ESP.restart();
  }
  else if (cmd.equalsIgnoreCase("GET GPS")) {
    char buf[64];
    snprintf(buf, sizeof(buf), "GPS|%.6f|%.6f|%d|%.1f",
      gps.location.lat(), gps.location.lng(), gps.satellites.value(), gps.speed.kmph());
    bleNotify(String(buf));
  }
}

void gpsTick() {
  // จำกัดการอ่านต่อลูป เพื่อไม่ให้ Block การทำงานอื่นเกินไป (Safety)
  int maxChars = 50;
  while (GPS.available() && maxChars-- > 0) {
    if (gps.encode(GPS.read())) {
      if (gps.speed.isValid()) {
        curSpeed = gps.speed.kmph();
        tLastSpeed = millis();
        if (curSpeed > 1.0 || tLastMove == 0) tLastMove = millis();
      }
    }
  }
}

void autoControl() {
  if (lastPos == 6) return;
  unsigned long now = millis();
  bool shouldClose = false; bool shouldOpen = false;

  if (tLastMove > 0 && now - tLastMove > STATIONARY_TIME) shouldClose = true;
  if (now - tLastSpeed < SPEED_TIMEOUT) {
    if (singleThreshEnabled) {
      if (curSpeed >= (vThresh + hyst * 0.5f)) shouldClose = true;
      if (curSpeed <= (vThresh - hyst * 0.5f)) shouldOpen = true;
    } else {
      if (curSpeed > vHigh) shouldClose = true;
      if (curSpeed <= vLow) shouldOpen = true;
    }
  }

  if (shouldClose && !autoMode) { executeMove(6); autoMode = true; }
  else if (shouldOpen && autoMode) { executeMove(1); autoMode = false; }
}

void buttonTick() {
  int reading = digitalRead(BUTTON_PIN);
  if (reading != lastButtonPhysState) lastDebounceTime = millis();
  if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
    if (reading != buttonLogicState) {
      buttonLogicState = reading;
      if (buttonLogicState == LOW) {
        if (lastPos == 6) { executeMove(1); autoMode = false; }
        else { executeMove(6); autoMode = true; }
      }
    }
  }
  lastButtonPhysState = reading;
}

void setup() {
  Serial.begin(115200);

  // ตั้งค่า Frequency ให้เหมาะกับ Servo ทั่วไป (50Hz)
  s1.setPeriodHertz(50);
  s2.setPeriodHertz(50);

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  // ADC Configuration for Voltage Sensing
  pinMode(34, INPUT);

  GPS.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  bleCommandToProcess[0] = '\0';
  loadEEP();

  // ตั้งค่าเริ่มต้นของ Servo (จะ Write ก่อน Attach ในฟังก์ชันนี้)
  currentAngle = angles[lastPos];
  attachServos();

  bleInit(); // เริ่ม BLE ด้วยชื่อ AEROTEQ Nano + MAC
}

void loop() {
  gpsTick(); buttonTick(); servoTick();

  if (bleCommandFlag) {
    String cmd;
    // ป้องกัน Race Condition เล็กน้อย
    noInterrupts();
    cmd = String((char*)bleCommandToProcess);
    bleCommandFlag = false;
    interrupts();
    handleCmd(cmd);
  }
  if (Serial.available()) handleCmd(Serial.readStringUntil('\n'));

  autoControl();

  // Periodic Notify (ส่งสถานะทุก 1 วินาที)
  static unsigned long lastNotify = 0;
  if (bleConnected && millis() - lastNotify > 1000) {
    lastNotify = millis();
    char msg[64];
    // Format: M(Pos)|Speed|Mode
    snprintf(msg, sizeof(msg), "M%d|%.1f|%s", lastPos, curSpeed, autoMode?"A":"M");
    bleNotify(String(msg));
  }

  // Hardware Telemetry (ส่งทุก 5 วินาที)
  static unsigned long lastHwNotify = 0;
  if (bleConnected && millis() - lastHwNotify > 5000) {
    lastHwNotify = millis();

    // --- REAL SENSOR READS ---
    // Temperature: Internal ESP32 Sensor (Celsius)
    float realTemp = temperatureRead();

    // Voltage: Read from GPIO 34 (Common ADC for battery/input)
    // Using analogReadMilliVolts for better precision, assuming 1/2 divider
    float realVolt = (analogReadMilliVolts(34) * 2.0) / 1000.0;

    char hwMsg[64];
    // Format: HW|Voltage|Temp|Heap
    snprintf(hwMsg, sizeof(hwMsg), "HW|%.1f|%.1f|%d", realVolt, realTemp, ESP.getFreeHeap());
    bleNotify(String(hwMsg));
  }
}
