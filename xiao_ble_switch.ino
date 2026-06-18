/*
  Xiao nRF52840 Sense Plus — BLE Bewegungs-Schalter + Vibration (DRV2605, LRA)  "XiaoSwitch"
  -----------------------------------------------------------------------------------------
  Xiao -> App : Bewegungsstärke (Notify), die App entscheidet per Schwelle.
  App  -> Xiao: Vibrationsbefehl (Write) -> DRV2605 treibt den LRA mit der Stärke.

  >>> Core / Bibliothek <<<
  Core "Seeeduino nRF52" (Bluefruit-basiert), FQBN: Seeeduino:nrf52:xiaonRF52840SensePlus.
  NICHT ArduinoBLE (sonst Linkerfehler "undefined reference to HCITransport").
  Bibliotheken:
    - bluefruit.h            (im Core enthalten)
    - Seeed Arduino LSM6DS3   (Bibliotheksverwalter)
    - Adafruit DRV2605        (Bibliotheksverwalter: "Adafruit DRV2605 Library")

  Datenmodell (identisch zur App):
    Service        19b10000-e8f2-537e-4f6c-d104768a1214
    Motion-Char    19b10001-...  (Read | Notify)  uint8 = Bewegungsstärke (g*100, 0..255)
    Vibe-Char      19b10002-...  (Write)          uint8 = Vibrationsstärke (0..255)

  >>> DRV2605 ANSCHLUSS (I2C, Adresse 0x5A) <<<
    DRV2605 VIN -> 3V3,  GND -> GND,  SDA -> SDA (D4),  SCL -> SCL (D5)
    LRA an die beiden Motor-Ausgänge des DRV2605-Breakouts.

  >>> ENERGIESPARMODI <<<
  Nach IDLE_TIMEOUT_MS ohne Bewegung geht der Sketch in systemOff (~0.4 µA).
  Der LSM6DS3 läuft dann im Wake-on-Motion-Modus (~7 µA, INT1 = P0.11 intern).
  Bewegt sich die Biene -> INT1 high -> nRF52840 wacht auf -> setup() läuft neu.
  GPREGRET=0x42 zeigt an, dass wir aus dem Sleep kommen (nicht Kaltstart).
*/

#include <bluefruit.h>
#include "LSM6DS3.h"
#include "Wire.h"
#include "Adafruit_DRV2605.h"
#include "mouse.h"


// ---- Pins ----
#define IMU_INT1_PIN  11   // P0.11, intern mit LSM6DS3 INT1 verdrahtet (kein extra Draht nötig)

// ---- Zeitkonstanten ----
const float         NOISE_FLOOR    = 0.05;    // g — darunter gilt es als Ruhe
const unsigned long SAMPLE_MS      = 40;      // ~25 Hz Abtastung
const unsigned long VIBE_MS        = 350;     // Pulsdauer je Befehl
const unsigned long BAT_MS         = 30000;   // Akku alle 30 s messen
const unsigned long IDLE_TIMEOUT_MS = 60000;  // 60 s ohne Bewegung -> systemOff

// ---- WOM-Register (LSM6DS3, nicht alle im Seeed-Header definiert) ----
#define REG_CTRL1_XL    0x10   // Accel Konfiguration
#define REG_CTRL2_G     0x11   // Gyro Konfiguration
#define REG_WAKE_UP_DUR 0x5C   // Wake-up Latenz (0 = sofort)
#define REG_WAKE_UP_THS 0x5B   // Wake-up Schwelle [5:0], bei ±2g: 1LSB = 31.25 mg
#define REG_MD1_CFG     0x5E   // INT1-Routing: Bit5 = Wake-up

// ---- GPREGRET Marker ----
#define SLEEP_MARKER 0x42   // in NRF_POWER->GPREGRET: wir kommen aus systemOff

// ---- Hardware-Objekte ----
LSM6DS3          imu(I2C_MODE, 0x6A);
Adafruit_DRV2605 drv;
bool             haveDrv = false;

// ---- UUIDs (Little-Endian) ----
const uint8_t MOTION_SERVICE_UUID[16] = {0x14,0x12,0x8a,0x76,0x04,0xd1,0x6c,0x4f,0x7e,0x53,0xf2,0xe8,0x00,0x00,0xb1,0x19};
const uint8_t MOTION_CHAR_UUID[16]    = {0x14,0x12,0x8a,0x76,0x04,0xd1,0x6c,0x4f,0x7e,0x53,0xf2,0xe8,0x01,0x00,0xb1,0x19};
const uint8_t VIBE_CHAR_UUID[16]      = {0x14,0x12,0x8a,0x76,0x04,0xd1,0x6c,0x4f,0x7e,0x53,0xf2,0xe8,0x02,0x00,0xb1,0x19};

BLEService        motionService(MOTION_SERVICE_UUID);
BLECharacteristic motionChar(MOTION_CHAR_UUID);
BLECharacteristic vibeChar(VIBE_CHAR_UUID);
BLEBas            blebas;

// ---- Akku ----
#ifndef PIN_VBAT
#define PIN_VBAT    (32u)
#endif
#ifndef VBAT_ENABLE
#define VBAT_ENABLE (14u)
#endif
#define VBAT_CAL  (1.000f)
unsigned long batNextAt = 0;

// ---- Zustand ----
float pax = 0, pay = 0, paz = 0;
bool  primed    = false;
bool  wasMoving = false;
unsigned long lastMoveAt   = 0;
unsigned long ledOffAt     = 0;
unsigned long vibeOffAt    = 0;
int  currentVibe = 0;
int  targetVibe  = 0;
unsigned long lastVibeTick = 0;

// ==========================================================================
// IMU-Konfiguration
// ==========================================================================

// Normalbetrieb: Gyro aus (spart ~0.9 mA), Accel 26 Hz ±2g
void configureIMU_Normal() {
  imu.writeRegister(REG_CTRL2_G,  0x00);   // Gyro power-down
  imu.writeRegister(REG_CTRL1_XL, 0x20);  // Accel: 26 Hz, ±2g
}

// Wake-on-Motion: IMU schläft intern, feuert INT1 bei Bewegung
void configureIMU_WOM() {
  imu.writeRegister(REG_CTRL2_G,     0x00);  // Gyro aus
  imu.writeRegister(REG_CTRL1_XL,    0x20);  // Accel 26 Hz (WOM funktioniert auf 26 Hz)
  imu.writeRegister(REG_WAKE_UP_DUR, 0x00);  // keine Latenz
  imu.writeRegister(REG_WAKE_UP_THS, 0x02);  // Schwelle ~62 mg (2 × 31.25 mg)
  imu.writeRegister(REG_MD1_CFG,     0x20);  // Wake-up auf INT1 routen
}

// ==========================================================================
// Sleep
// ==========================================================================

void goToSleep() {
  Serial.print("Idle seit "); Serial.print(IDLE_TIMEOUT_MS / 1000); Serial.println(" s → systemOff");
  Serial.flush();

  // Motor sicher aus
  if (haveDrv) { drv.setRealtimeValue(0); currentVibe = 0; targetVibe = 0; }

  // LED aus
  digitalWrite(LED_BUILTIN, HIGH);

  // IMU auf WOM umstellen
  configureIMU_WOM();
  delay(10);   // kurz warten bis WOM aktiv

  // GPIO für systemOff-Wakeup: INT1 weckt auf steigende Flanke (HIGH)
  nrf_gpio_cfg_sense_input(IMU_INT1_PIN, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_SENSE_HIGH);

  // Marker setzen: nach dem Wakeup wissen wir, dass es ein Sleep-Wakeup war
  NRF_POWER->GPREGRET = SLEEP_MARKER;

  // BLE sauber beenden
  if (Bluefruit.connected()) Bluefruit.disconnect(Bluefruit.connHandle());
  Bluefruit.Advertising.stop();
  delay(50);

  // Ab hier: MCU schläft, nur IMU-Interrupt kann aufwecken (~0.4 + 7 µA)
  NRF_POWER->SYSTEMOFF = 1;
  while (1);   // nie erreicht; beruhigt den Compiler
}

// ==========================================================================
// Akku
// ==========================================================================

uint8_t lipoPercent(float v) {
  static const float V[] = {3.30,3.60,3.70,3.75,3.79,3.83,3.87,3.92,3.98,4.06,4.15};
  static const float P[] = {   0,   5,  15,  25,  40,  50,  60,  70,  80,  90, 100};
  const int n = sizeof(V)/sizeof(V[0]);
  if (v <= V[0])   return 0;
  if (v >= V[n-1]) return 100;
  for (int i = 1; i < n; i++) {
    if (v < V[i]) {
      float f = (v - V[i-1]) / (V[i] - V[i-1]);
      return (uint8_t)(P[i-1] + f * (P[i] - P[i-1]) + 0.5f);
    }
  }
  return 100;
}

float readBatteryVoltage() {
  digitalWrite(VBAT_ENABLE, LOW);
  delay(5);
  uint32_t sum = 0;
  for (int i = 0; i < 16; i++) { sum += analogRead(PIN_VBAT); delayMicroseconds(150); }
  float raw  = sum / 16.0f;
  float vPin = raw * (3.0f / 4096.0f);
  return vPin * (1510.0f / 510.0f) * VBAT_CAL;
}

uint8_t readBatteryPercent() {
  float v = readBatteryVoltage();
  uint8_t pct = lipoPercent(v);
  Serial.print("VBAT = "); Serial.print(v, 3); Serial.print(" V  -> "); Serial.print(pct); Serial.println(" %");
  return pct;
}

// ==========================================================================
// BLE Callbacks
// ==========================================================================

void vibe_write_cb(uint16_t conn_handle, BLECharacteristic* chr, uint8_t* data, uint16_t len) {
  (void)conn_handle; (void)chr;
  if (len < 1 || !haveDrv) return;
  uint8_t strength = data[0];
  if (strength == 0) { targetVibe = 0; vibeOffAt = 0; return; }
  targetVibe = map(strength, 1, 255, 30, 127);
  vibeOffAt  = millis() + VIBE_MS;
}

void connect_cb(uint16_t handle) {
  (void)handle;
  digitalWrite(LED_BUILTIN, LOW); ledOffAt = millis() + 300;
  lastMoveAt = millis();   // kein sofortiger Sleep nach Connect
  Serial.println("Verbunden");
}

void disconnect_cb(uint16_t handle, uint8_t reason) {
  (void)handle; (void)reason;
  digitalWrite(LED_BUILTIN, HIGH);
  targetVibe = 0; vibeOffAt = 0;   // Motor aus
  Serial.println("Getrennt – werbe wieder...");
}

void startAdv() {
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(motionService);
  Bluefruit.ScanResponse.addName();
  Bluefruit.Advertising.restartOnDisconnect(true);
  // Sparsameres Intervall: 100 ms–1 s statt 20–150 ms
  // Wer schnell verbinden will: setFastTimeout(30) liefert 30 s mit kleinerem Intervall
  Bluefruit.Advertising.setInterval(160, 1600);   // 100 ms / 1 s
  Bluefruit.Advertising.setFastTimeout(30);        // erste 30 s schneller
  Bluefruit.Advertising.start(0);
}

// ==========================================================================
// setup
// ==========================================================================

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT); digitalWrite(LED_BUILTIN, HIGH);

  // Wakeup-Ursache prüfen
  bool fromSleep = (NRF_POWER->GPREGRET == SLEEP_MARKER);
  NRF_POWER->GPREGRET = 0;   // löschen
  if (fromSleep) Serial.println("Wakeup aus systemOff (WOM)");
  else           Serial.println("Kaltstart");

  if (imu.begin() != 0) {
    // IMU nicht gefunden: schnelles Dauerblinken
    while (1) { digitalWrite(LED_BUILTIN, LOW); delay(80); digitalWrite(LED_BUILTIN, HIGH); delay(80); }
  }
  configureIMU_Normal();   // Gyro aus, Accel 26 Hz

  // DRV2605
  if (drv.begin()) {
    haveDrv = true;
    drv.writeRegister8(0x01, 0x80);   // Hard-Reset
    delay(50);
    drv.useERM();
    drv.selectLibrary(1);
    drv.setMode(DRV2605_MODE_REALTIME);
    drv.setRealtimeValue(0);
  } else {
    Serial.println("DRV2605 nicht gefunden – Vibration deaktiviert.");
  }

  // Akku
  pinMode(VBAT_ENABLE, OUTPUT);
  digitalWrite(VBAT_ENABLE, LOW);
  analogReference(AR_INTERNAL_3_0);
  analogReadResolution(12);

  // BLE
  Bluefruit.begin();
  Bluefruit.setTxPower(0);                        // war 4 — im Zimmer reicht 0, spart ~1 mA
  Bluefruit.setName("XiaoSwitch");
  Bluefruit.Periph.setConnectCallback(connect_cb);
  Bluefruit.Periph.setDisconnectCallback(disconnect_cb);
  Bluefruit.Periph.setConnInterval(16, 48);

  blebas.begin();
  blebas.write(readBatteryPercent());

  motionService.begin();

  motionChar.setProperties(CHR_PROPS_NOTIFY | CHR_PROPS_READ);
  motionChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  motionChar.setFixedLen(1);
  motionChar.begin();
  motionChar.write8(0);

  vibeChar.setProperties(CHR_PROPS_WRITE | CHR_PROPS_WRITE_WO_RESP);
  vibeChar.setPermission(SECMODE_NO_ACCESS, SECMODE_OPEN);
  vibeChar.setFixedLen(1);
  vibeChar.setWriteCallback(vibe_write_cb);
  vibeChar.begin();

  startAdv();
  lastMoveAt = millis();   // Idle-Timer starten
  Serial.println("XiaoSwitch bereit – werbe...");
}

// ==========================================================================
// loop
// ==========================================================================

void loop() {
  unsigned long now = millis();

  // --- Vibe Fading ---
  if (now - lastVibeTick > 10) {
    lastVibeTick = now;
    if (currentVibe < targetVibe) {
      currentVibe += 8; if (currentVibe > targetVibe) currentVibe = targetVibe;
      drv.setRealtimeValue(currentVibe);
    } else if (currentVibe > targetVibe) {
      currentVibe -= 4; if (currentVibe < targetVibe) currentVibe = targetVibe;
      drv.setRealtimeValue(currentVibe);
    }
  }

  // --- Timers ---
  if (vibeOffAt && now >= vibeOffAt) { targetVibe = 0; vibeOffAt = 0; }
  if (ledOffAt  && now >= ledOffAt)  { digitalWrite(LED_BUILTIN, HIGH); ledOffAt = 0; }
  if (now >= batNextAt) { batNextAt = now + BAT_MS; blebas.write(readBatteryPercent()); }

  // --- IMU auslesen (nur wenn verbunden) ---
  if (Bluefruit.connected()) {
    float ax = imu.readFloatAccelX();
    float ay = imu.readFloatAccelY();
    float az = imu.readFloatAccelZ();

    if (!primed) { pax = ax; pay = ay; paz = az; primed = true; }
    else {
      float dx = ax - pax, dy = ay - pay, dz = az - paz;
      float delta = sqrt(dx * dx + dy * dy + dz * dz);
      pax = ax; pay = ay; paz = az;

      if (delta > NOISE_FLOOR) {
        int v = (int)(delta * 100.0 + 0.5);
        if (v > 255) v = 255;
        motionChar.notify8((uint8_t)v);
        wasMoving  = true;
        lastMoveAt = now;   // Idle-Timer zurücksetzen
      } else if (wasMoving) {
        motionChar.notify8(0);
        wasMoving = false;
      }
    }
  } else {
    primed = false;
  }

  // --- Idle-Timeout: nach 60 s ohne Bewegung schlafen ---
  if (now - lastMoveAt > IDLE_TIMEOUT_MS) {
    goToSleep();
  }

  delay(SAMPLE_MS);   // CPU schläft (sd_app_evt_wait), BLE läuft im Hintergrund
}
