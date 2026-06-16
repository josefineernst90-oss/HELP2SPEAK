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

  Vibration: LRA im Echtzeit-Modus (RTP). Die Stärke 0..255 von der App wird direkt
  als Amplitude (0..100 %) gesetzt – stufenlos regelbar. Nach VIBE_MS schaltet der
  Puls automatisch wieder ab.
*/

#include <bluefruit.h>
#include "LSM6DS3.h"
#include "Wire.h"
#include "Adafruit_DRV2605.h"

LSM6DS3          imu(I2C_MODE, 0x6A);
Adafruit_DRV2605 drv;
bool             haveDrv = false;

// ---- UUIDs als Byte-Array in LITTLE-ENDIAN (rückwärts) ----
const uint8_t MOTION_SERVICE_UUID[16] = {0x14,0x12,0x8a,0x76,0x04,0xd1,0x6c,0x4f,0x7e,0x53,0xf2,0xe8,0x00,0x00,0xb1,0x19};
const uint8_t MOTION_CHAR_UUID[16]    = {0x14,0x12,0x8a,0x76,0x04,0xd1,0x6c,0x4f,0x7e,0x53,0xf2,0xe8,0x01,0x00,0xb1,0x19};
const uint8_t VIBE_CHAR_UUID[16]      = {0x14,0x12,0x8a,0x76,0x04,0xd1,0x6c,0x4f,0x7e,0x53,0xf2,0xe8,0x02,0x00,0xb1,0x19};

BLEService        motionService(MOTION_SERVICE_UUID);
BLECharacteristic motionChar(MOTION_CHAR_UUID);
BLECharacteristic vibeChar(VIBE_CHAR_UUID);
BLEBas            blebas;        // Standard Battery Service (0x180F / 0x2A19)

// ---- Akku-Messung (Xiao nRF52840 Sense) ----
// PIN_VBAT (P0.31) = ADC-Pin, VBAT_ENABLE (P0.14) = Messpfad (LOW = aktiv).
// Beide sind im Seeed-XIAO-nRF52840-Core definiert; Fallback nur zur Sicherheit.
#ifndef PIN_VBAT
#define PIN_VBAT     (32u)
#endif
#ifndef VBAT_ENABLE
#define VBAT_ENABLE  (14u)
#endif
const unsigned long BAT_MS = 30000;   // alle 30 s messen
// Kalibrierfaktor: 1.0 = unkorrigiert. Falls Multimeter z.B. 4,20 V zeigt, der
// Serial-Monitor aber 4,11 V meldet -> VBAT_CAL = 4.20 / 4.11 = 1.022 setzen.
#define VBAT_CAL  (1.000f)
unsigned long batNextAt = 0;

// ---- Bewegungserkennung ----
const float         NOISE_FLOOR = 0.05;   // g — darunter gilt es als Ruhe
const unsigned long SAMPLE_MS   = 40;     // ~25 Hz Abtastung
float pax = 0, pay = 0, paz = 0;
bool  primed = false;
bool  wasMoving = false;                  // damit am Bewegungsende einmal "0" gemeldet wird

// ---- Vibration ----
const unsigned long VIBE_MS = 350;        // Pulsdauer je Befehl
unsigned long vibeOffAt = 0;

int currentVibe = 0;
int targetVibe = 0;
unsigned long lastVibeTick = 0;

// ---- Status-LED: nur kurzer Blink bei Connect, danach aus (spart Strom) ----
unsigned long ledOffAt = 0;

void blinkForever(int onMs, int offMs) {
  while (1) { digitalWrite(LED_BUILTIN, LOW); delay(onMs); digitalWrite(LED_BUILTIN, HIGH); delay(offMs); }
}

// Batteriespannung lesen und grob in Prozent (LiPo 3,3–4,2 V) umrechnen
// LiPo-Ladezustand aus Spannung (stückweise lineare Kennlinie, grob aber realistisch)
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
  digitalWrite(VBAT_ENABLE, LOW);                  // Messpfad aktiv
  delay(5);
  uint32_t sum = 0;
  for (int i = 0; i < 16; i++) { sum += analogRead(PIN_VBAT); delayMicroseconds(150); }
  float raw  = sum / 16.0f;                        // gemittelt
  float vPin = raw * (3.0f / 4096.0f);             // Spannung am ADC-Pin (3,0 V Ref, 12 Bit)
  return vPin * (1510.0f / 510.0f) * VBAT_CAL;     // Teiler zurückrechnen + Kalibrierung
}

uint8_t readBatteryPercent() {
  float v = readBatteryVoltage();
  uint8_t pct = lipoPercent(v);
  Serial.print("VBAT = "); Serial.print(v, 3); Serial.print(" V  -> "); Serial.print(pct); Serial.println(" %");
  return pct;
}

void vibeStop() { 
  targetVibe = 0; // Fade-Out einleiten
  vibeOffAt = 0; 
}

// App -> Xiao: Vibrationsbefehl (1 Byte Stärke 0..255 = Amplitude)
void vibe_write_cb(uint16_t conn_handle, BLECharacteristic* chr, uint8_t* data, uint16_t len) {
  (void)conn_handle; (void)chr;
  if (len < 1 || !haveDrv) return;
  
  uint8_t strength = data[0];
  if (strength == 0) { vibeStop(); return; }
  
  // App-Werte übersetzen und als neues Ziel setzen
  targetVibe = map(strength, 1, 255, 30, 127);
  vibeOffAt = millis() + VIBE_MS;
}

void connect_cb(uint16_t handle) {
  (void)handle;
  digitalWrite(LED_BUILTIN, LOW); ledOffAt = millis() + 300;   // kurzer Bestätigungs-Blink
  Serial.println("Verbunden");
}
void disconnect_cb(uint16_t handle, uint8_t reason) {
  (void)handle; (void)reason;
  digitalWrite(LED_BUILTIN, HIGH);        // LED aus
  vibeStop();                             // Motor sicher aus
  Serial.println("Getrennt – werbe wieder...");
}

void startAdv() {
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(motionService);
  Bluefruit.ScanResponse.addName();
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);
  Bluefruit.Advertising.setFastTimeout(30);
  Bluefruit.Advertising.start(0);
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT); digitalWrite(LED_BUILTIN, HIGH);   // aus

  if (imu.begin() != 0) blinkForever(80, 80);   // IMU nicht gefunden

  // DRV2605 starten (Vibration). Fehlt er, läuft der Rest trotzdem.
if (drv.begin()) {
    haveDrv = true;
    
    // 1. KOMPLETTER HARDWARE-RESET (löscht die alten LRA-Werte aus dem Speicher!)
    drv.writeRegister8(0x01, 0x80); 
    delay(50); // Kurz warten, bis der Reset durch ist
    
    // 2. Standard ERM-Motor einstellen
    drv.useERM();
    drv.selectLibrary(1);
    
    // 3. Echtzeit-Modus (weckt den Chip nach dem Reset automatisch auf)
    drv.setMode(DRV2605_MODE_REALTIME);
    drv.setRealtimeValue(0); 
  } else {
    Serial.println("DRV2605 nicht gefunden – Vibration deaktiviert.");
  }

  // Akku-Messung vorbereiten
  pinMode(VBAT_ENABLE, OUTPUT);
  digitalWrite(VBAT_ENABLE, LOW);            // dauerhaft aktiv (nie HIGH während Laden -> schützt P0.31)
  analogReference(AR_INTERNAL_3_0);          // 3,0 V Referenz
  analogReadResolution(12);                  // 0..4095

  Bluefruit.begin();
  Bluefruit.setTxPower(4);                       // Akku: ggf. auf 0 senken (kürzere Reichweite)
  Bluefruit.setName("XiaoSwitch");
  Bluefruit.Periph.setConnectCallback(connect_cb);
  Bluefruit.Periph.setDisconnectCallback(disconnect_cb);
  Bluefruit.Periph.setConnInterval(16, 48);      // 20–60 ms: etwas Akku sparen, Latenz bleibt gut

  blebas.begin();                                 // Battery Service
  blebas.write(readBatteryPercent());

  motionService.begin();                          // Service VOR seinen Characteristics

  motionChar.setProperties(CHR_PROPS_NOTIFY | CHR_PROPS_READ);
  motionChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  motionChar.setFixedLen(1);
  motionChar.begin();
  motionChar.write8(0);

  vibeChar.setProperties(CHR_PROPS_WRITE | CHR_PROPS_WRITE_WO_RESP);
  vibeChar.setPermission(SECMODE_NO_ACCESS, SECMODE_OPEN);   // nur schreibbar
  vibeChar.setFixedLen(1);
  vibeChar.setWriteCallback(vibe_write_cb);
  vibeChar.begin();

  startAdv();
  Serial.println("XiaoSwitch bereit – werbe...");
}

void loop() {
  unsigned long now = millis();

  // --- NEU: Sanftes Fading (Fade-In und Fade-Out) ---
  if (now - lastVibeTick > 10) { // Alle 10 Millisekunden anpassen
    lastVibeTick = now;
    
    if (currentVibe < targetVibe) {
      currentVibe += 8; // Geschwindigkeit beim Hochfahren (Fade-In)
      if (currentVibe > targetVibe) currentVibe = targetVibe;
      drv.setRealtimeValue(currentVibe);
    } 
    else if (currentVibe > targetVibe) {
      currentVibe -= 4; // Geschwindigkeit beim Ausklingen (Fade-Out, etwas sanfter)
      if (currentVibe < targetVibe) currentVibe = targetVibe;
      drv.setRealtimeValue(currentVibe);
    }
  }

  // Vibration nach Pulsdauer abschalten
  if (vibeOffAt && now >= vibeOffAt) vibeStop();
  // Connect-Blink beenden
  if (ledOffAt && now >= ledOffAt) { digitalWrite(LED_BUILTIN, HIGH); ledOffAt = 0; }
  // Akku messen und melden (alle 30 s)
  if (now >= batNextAt) { batNextAt = now + BAT_MS; blebas.write(readBatteryPercent()); }

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
        wasMoving = true;
      } else if (wasMoving) {
        motionChar.notify8(0);   // einmal Ruhe melden (Pegelbalken/Stille-Erkennung)
        wasMoving = false;
      }
    }
  } else {
    primed = false;
  }

  delay(SAMPLE_MS);   // gibt die CPU frei -> nRF schläft zwischen Messungen (FreeRTOS), spart deutlich Strom
}
