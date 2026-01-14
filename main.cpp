#include <ESP32Servo.h>
#include <TinyGPS++.h>
#include <EEPROM.h>
#include "BLEDevice.h"
#include "BLEServer.h"
#include "BLEUtils.h"
#include "BLE2902.h"

// ================= CONFIGURATION =================
const int SERVO1_PIN = 13;
const int SERVO2_PIN = 12;
const int BUTTON_PIN = 10;
const int GPS_RX_PIN = 16;
const int GPS_TX_PIN = 17;

const unsigned long SERVO_MOVE_TIME = 1000; // เวลาให้ servo หมุน (ms) ก่อนตัดไฟ
const unsigned long STATIONARY_TIME = 120000;
const unsigned long SPEED_TIMEOUT   = 5000;
const unsigned long GPS_OUT_MS      = 2000;
const unsigned long DEBOUNCE_DELAY  = 50;
const int EEPROM_SIZE = 512;

// ================= GLOBALS =================
TinyGPSPlus gps;
HardwareSerial GPS(2);
Servo s1, s2;

// --- Servo State Machine ---
bool isServoMoving = false;
unsigned long servoMoveStart = 0;

// --- Button Vars ---
int buttonLogicState = HIGH;
int lastButtonPhysState = HIGH;
unsigned long lastDebounceTime = 0;

// --- Logic Vars ---
int angles[7] = {0, 40, 80, 110, 140, 160, 180}; // Index 1..6
int currentAngle = 0;
int lastPos = 6; // Default Closed
bool attached = false;

// Speed Control
float vHigh = 50, vLow = 50;
float curSpeed = 0;
bool singleThreshEnabled = false;
float vThresh = 50;
float hyst = 0;

bool autoMode = true;
unsigned long tLastMove = 0, tLastSpeed = 0, tLastGpsOut = 0;

// Invert
bool invertS1 = false;
bool invertS2 = true;

// BLE
volatile bool bleCommandFlag = false;
volatile char bleCommandToProcess[128];
BLEServer* pServer = nullptr;
BLECharacteristic* pChar = nullptr;
bool bleConnected = false;

// ================= EEPROM MAP =================
const int ADDR_LAST_POS = 24;
const int ADDR_HIGH     = 28;
const int ADDR_LOW      = 32;
const int ADDR_VTHRESH  = 36;
const int ADDR_STEN     = 40;
const int ADDR_HYST     = 44;
const int ADDR_INV_S1   = 48;
const int ADDR_INV_S2   = 52;

// ================= HELPER FUNCTIONS =================

int clampAngle(int a, int def) {
  return (a < 0 || a > 180) ? def : a;
}

void attachServos() {
  if (!attached) {
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
    Serial.println("[Servo] Power cut (Detached)");
  }
}

// *** IMPROVEMENT 1: Non-blocking Move Logic ***
// ฟังก์ชันนี้สั่งหมุน แล้วจบเลย (ไม่รอ delay)
void startMoveTo(int target) {
  if (target == currentAngle && !attached) return;
  
  attachServos();
  
  int a1 = invertS1 ? 180 - target : target;
  int a2 = invertS2 ? 180 - target : target;
  s1.write(a1);
  s2.write(a2);
  
  currentAngle = target;
  
  // เริ่มจับเวลาสำหรับตัดไฟ
  isServoMoving = true;
  servoMoveStart = millis(); 
}

// เรียกใน Loop เพื่อเช็คว่าครบเวลาหรือยัง
void servoTick() {
  if (isServoMoving) {
    if (millis() - servoMoveStart > SERVO_MOVE_TIME) {
      detachServos();
      isServoMoving = false;
    }
  }
}

// *** IMPROVEMENT 2: Centralized Execution ***
// ทุกคำสั่งเปลี่ยนตำแหน่งต้องผ่านฟังก์ชันนี้ เพื่อให้ state ของปุ่มกด sync กันเสมอ
void executeMove(int posIndex) {
  if (posIndex < 1 || posIndex > 6) return;
  
  lastPos = posIndex;
  
  // บันทึกสถานะล่าสุด
  EEPROM.writeInt(ADDR_LAST_POS, lastPos);
  EEPROM.commit();
  
  Serial.printf("[Action] Moving to M%d (%d deg)\n", lastPos, angles[lastPos]);
  startMoveTo(angles[lastPos]);
}

// ================= EEPROM & INIT =================
inline int addrAngle(int pos) { return (pos - 1) * 4; }

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

  // Angles
  for (int i = 1; i <= 6; i++) {
    int v = EEPROM.readInt(addrAngle(i));
    if (v != 0 || i == 6) angles[i] = clampAngle(v, angles[i]);
  }

  lastPos = EEPROM.readInt(ADDR_LAST_POS);
  if (lastPos < 1 || lastPos > 6) lastPos = 6;

  float vh = EEPROM.readFloat(ADDR_HIGH);
  float vl = EEPROM.readFloat(ADDR_LOW);
  if (vh > 0) vHigh = vh; 
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

  Serial.printf("Loaded: M%d, Invert(%d,%d)\n", lastPos, invertS1, invertS2);
}

// ================= LOGIC & SENSORS =================

void gpsTick() {
  while (GPS.available()) {
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
  if (lastPos == 6) return; // ถ้าพับอยู่แล้ว ไม่ต้อง auto
  
  unsigned long now = millis();
  bool shouldClose = false;
  bool shouldOpen = false;

  // 1. กฎจอดนิ่ง
  if (tLastMove > 0 && now - tLastMove > STATIONARY_TIME) {
    shouldClose = true;
  }

  // 2. กฎความเร็ว
  if (now - tLastSpeed < SPEED_TIMEOUT) {
    if (singleThreshEnabled) {
      float up   = vThresh + (hyst * 0.5f);
      float down = vThresh - (hyst * 0.5f);
      if (curSpeed >= up) shouldClose = true;
      if (curSpeed <= down) shouldOpen = true;
    } else {
      if (curSpeed > vHigh) shouldClose = true;
      if (curSpeed <= vLow) shouldOpen = true;
    }
  }

  if (shouldClose && !autoMode) {
    Serial.println("[Auto] Closing...");
    executeMove(6); 
    autoMode = true;
  } else if (shouldOpen && autoMode) {
    // กลับไปค่าเดิมก่อนพับ (ถ้าจำไม่ได้ ให้ไป M1)
    // ในที่นี้เราใช้ logic ง่ายๆ คือเปิดไป M1 หรือตำแหน่งที่เหมาะสม
    // แต่ code เดิมคือ openToLast ซึ่ง lastPos คือตำแหน่งปัจจุบัน
    // ดังนั้น ถ้าจะเปิด ต้องรู้ว่าเปิดไปไหน? สมมติเปิดไป M1 ละกันถ้าไม่มีตัวจำก่อนหน้า
    // *Logic เดิมของคุณคือ openToLast แต่มันจะ work ก็ต่อเมื่อ lastPos ไม่ใช่ 6
    // ซึ่งขัดแย้งกันนิดหน่อย ขอแก้เป็น -> เปิดไป M1
    Serial.println("[Auto] Opening...");
    executeMove(1); 
    autoMode = false;
  }
}

void handleCmd(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;
  Serial.println("> CMD: " + cmd);

  if (toupper(cmd[0]) == 'M') {
    int pos = cmd.substring(1).toInt();
    if (pos >= 1 && pos <= 6) {
      executeMove(pos);
      autoMode = false;
    }
  } 
  else if (cmd.equalsIgnoreCase("OPEN")) {
    executeMove(1); // เปิดไป M1
    autoMode = false;
  }
  else if (cmd.equalsIgnoreCase("CLOSE")) {
    executeMove(6); // พับ
    autoMode = false; // ปิดแล้วก็ manual mode เลย
  }
  else if (cmd.equalsIgnoreCase("AUTO")) {
    autoMode = true;
    Serial.println("Mode: AUTO");
  }
  else if (cmd.equalsIgnoreCase("STATUS")) {
     Serial.printf("POS: M%d, SPD: %.1f, MODE: %s\n", lastPos, curSpeed, autoMode?"AUTO":"MANUAL");
  }
  // ... (คำสั่ง SET ต่างๆ คงเดิม ใช้ saveSettings() ตอนท้ายได้)
}

// ================= BLE =================
#define BLE_SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define BLE_CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

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
  BLEDevice::init("ESP32S3_Mirror");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(BLE_SERVICE_UUID);
  pChar = pService->createCharacteristic(BLE_CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);
  pChar->addDescriptor(new BLE2902());
  pChar->setCallbacks(new MyCharCallbacks());
  pService->start();
  pServer->getAdvertising()->start();
}

void bleNotifyTick() {
  static unsigned long lastBLE = 0;
  if (!bleConnected || millis() - lastBLE < 1000) return;
  lastBLE = millis();
  char msg[64];
  snprintf(msg, sizeof(msg), "M%d|%.1f|%s", lastPos, curSpeed, autoMode?"A":"M");
  pChar->setValue((uint8_t*)msg, strlen(msg));
  pChar->notify();
}

// ================= BUTTON =================
void buttonTick() {
  int reading = digitalRead(BUTTON_PIN);
  if (reading != lastButtonPhysState) lastDebounceTime = millis();

  if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
    if (reading != buttonLogicState) {
      buttonLogicState = reading;
      if (buttonLogicState == LOW) { // Pressed
        Serial.println("[Button] Pressed");
        
        // *** IMPROVEMENT 2: Smart Toggle ***
        // เช็คจากสถานะปัจจุบัน (lastPos) แทนการใช้ตัวแปร isNextPress แยก
        if (lastPos == 6) {
          // ถ้าปัจจุบันปิดอยู่ -> ให้เปิด (M1)
          executeMove(1);
          autoMode = false;
        } else {
          // ถ้าปัจจุบันเปิดอยู่ (M1..M5) -> ให้ปิด (M6)
          executeMove(6);
          autoMode = true; // กดปิดมักจะอยากให้เข้า Auto หรือไม่? ปรับได้ตามชอบ
        }
      }
    }
  }
  lastButtonPhysState = reading;
}

// ================= MAIN =================
void setup() {
  Serial.begin(115200);
  
  s1.setPeriodHertz(50);
  s2.setPeriodHertz(50);
  
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  
  GPS.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  bleCommandToProcess[0] = '\0';
  loadEEP();
  
  // Init move
  attachServos();
  startMoveTo(angles[lastPos]);
  
  bleInit();
  Serial.println("System Ready.");
}

void loop() {
  gpsTick();
  buttonTick();
  servoTick(); // <-- สำคัญ: คอยตัดไฟ servo เมื่อครบเวลา
  
  // Handle BLE Command
  if (bleCommandFlag) {
    String cmd;
    noInterrupts();
    cmd = String((char*)bleCommandToProcess);
    bleCommandFlag = false;
    interrupts();
    handleCmd(cmd);
  }

  // Handle Serial Debug Command
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    handleCmd(line);
  }

  autoControl();
  bleNotifyTick();
  
  // Debug Info (GPS)
  if (millis() - tLastGpsOut > GPS_OUT_MS) {
     Serial.printf("[Info] GPS Fix: %d Sat, %.6f,%.6f\n", gps.satellites.value(), gps.location.lat(), gps.location.lng());
     tLastGpsOut = millis();
  }
}
