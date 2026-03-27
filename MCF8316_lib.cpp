/**
 * ============================================================
 *  MCF8316C-Q1  –  Sensorless FOC BLDC Driver
 *  Teensy 4.0  |  PlatformIO / Arduino Framework
 * ============================================================
 *
 *  Hardware-Verdrahtung (Teensy 4.0):
 *    Wire1 SDA  →  Pin 17  →  MCF8316 SDA
 *    Wire1 SCL  →  Pin 16  →  MCF8316 SCL
 *    PWM-Out    →  Pin 33  →  MCF8316 SPEED/WAKE
 *    Pull-ups (4,7 kΩ) auf SDA und SCL nach 3,3 V !
 *    nFAULT-LED extern verdrahtet (hardwired lt. Aufbau)
 *
 *  I2C-Protokoll (aus Datasheet SLLSFV2 + SLLA662):
 *    - Eigenes 24-Bit Control-Word-Format (kein Standard-I2C-Register-Schema)
 *    - TARGET_ID  = 0x01 (Default-Adresse nach Power-On)
 *    - CRC        = deaktiviert (einfacherer Betrieb)
 *    - DLEN       = 01b  →  32-Bit Daten pro Transaktion
 *    - MEM_SEC    = 0x0 / MEM_PAGE = 0x0  (alle nutzbaren Register)
 *    - Datenbytes werden LSB-zuerst übertragen
 *    - MCF8316 unterstützt Clock-Stretching (Timeout 4,66 ms)
 *      → übernimmt das nötige 100 µs Inter-Byte-Delay automatisch
 *
 *  Steuerprinzip:
 *    SPEED_MODE = 01b  (PIN_CONFIG1, EEPROM-Shadow 0x0A4)
 *    → Motor-Geschwindigkeit über PWM-Duty-Cycle auf Pin 33
 *    → 0 % Duty  = Motor aus / Standby
 *    → 100 % Duty = Maximale Drehzahl
 *
 *  Testschleife:
 *    10 Sekunden Motor EIN (50 %)  →  10 Sekunden Motor AUS  →  ...
 *
 * ============================================================
 */

#include <Arduino.h>
#include <Wire.h>

// ─────────────────────────────────────────────────────────────
//  Hardware-Konfiguration
// ─────────────────────────────────────────────────────────────
constexpr uint8_t  MCF_I2C_ADDR   = 0x01;    // TARGET_ID (Default = 0x01)
constexpr uint8_t  PWM_PIN        = 33;       // Teensy 4.0 PWM-Ausgang
constexpr uint32_t PWM_FREQ_HZ    = 20000;    // 20 kHz PWM (gut für BLDC)
constexpr uint32_t I2C_CLOCK_HZ   = 100000;   // 100 kHz  (Standard Mode)
                                               // Clock-Stretching deckt 100µs-Delay ab
constexpr uint8_t  I2C_RETRIES    = 5;        // Anzahl NACK-Retries (lt. Datasheet)
constexpr uint32_t RETRY_DELAY_MS = 5;        // Wartezeit zwischen Retries

// ─────────────────────────────────────────────────────────────
//  Register-Adressen  (MEM_ADDR-Teil, 12-Bit)
//  Vollständige Adresse: 0x000000 + Offset
// ─────────────────────────────────────────────────────────────

//  EEPROM Shadow Register  (Datasheet 7.7, Offset A4h)
//  Enthält SPEED_MODE – wirkt sofort auf den Shadow-RAM,
//  kein EEPROM-Brennvorgang nötig für reine Laufzeit-Konfiguration
constexpr uint16_t REG_PIN_CONFIG1       = 0x0A4;

//  EEPROM-Write-Trigger  (Datasheet 7.6.1.1, Schritt 18)
//  Nur benötigt wenn EEPROM dauerhaft beschrieben werden soll
constexpr uint16_t REG_EEPROM_CTRL       = 0x0EA;

//  RAM – Fault-Status Register  (Datasheet 7.8.1, Offset E0h / E2h)
constexpr uint16_t REG_GD_FAULT_STATUS  = 0x0E0;  // Gate Driver Faults
constexpr uint16_t REG_CT_FAULT_STATUS  = 0x0E2;  // Controller Faults

// ─────────────────────────────────────────────────────────────
//  Registerwerte
// ─────────────────────────────────────────────────────────────

// PIN_CONFIG1: Nur SPEED_MODE Bits [1:0] = 01b (PWM-Modus)
// Alle anderen Bits bleiben 0 (Default). Wert: 0x00000001
//   Bit 1:0  SPEED_MODE = 01b  → PWM-Duty-Cycle auf SPEED-Pin
constexpr uint32_t PIN_CONFIG1_PWM_MODE = 0x00000001;

// EEPROM-Write-Trigger: schreibt Shadow-Register dauerhaft in EEPROM
// (Nur verwenden wenn Einstellungen nach Power-Cycle erhalten bleiben sollen!)
constexpr uint32_t EEPROM_WRITE_KEY     = 0x8A500000;

// ─────────────────────────────────────────────────────────────
//  Control-Word Aufbau
//  Format (24-Bit):
//   [23]    OP_R/W   : 0 = Schreiben, 1 = Lesen
//   [22]    CRC_EN   : 0 = CRC deaktiviert
//   [21:20] DLEN     : 00=16bit, 01=32bit, 10=64bit
//   [19:16] MEM_SEC  : 0x0 für alle nutzbaren Register
//   [15:12] MEM_PAGE : 0x0 für alle nutzbaren Register
//   [11:0]  MEM_ADDR : Registeradresse (untere 12 Bit)
// ─────────────────────────────────────────────────────────────
static uint32_t buildControlWord(bool isRead, uint16_t addr) {
  uint32_t cw = 0;
  cw |= (isRead ? 1UL : 0UL) << 23;   // OP_R/W
  // Bit 22: CRC_EN = 0 (CRC deaktiviert)
  cw |= 0b01UL << 20;                  // DLEN = 01b → 32-Bit Daten
  // Bits 19:16: MEM_SEC = 0x0
  // Bits 15:12: MEM_PAGE = 0x0
  cw |= (uint32_t)(addr & 0x0FFF);     // MEM_ADDR (12-Bit)
  return cw;
}

// ─────────────────────────────────────────────────────────────
//  I2C: 32-Bit Schreiben
//  Sequenz (Datasheet 7.6.2.2):
//    START → [TARGET_ID + W=0] → ACK
//    → CW[23:16] → ACK → CW[15:8] → ACK → CW[7:0] → ACK
//    → DB0(LSB) → ACK → DB1 → ACK → DB2 → ACK → DB3(MSB) → ACK
//    → STOP
// ─────────────────────────────────────────────────────────────
static bool i2cWrite32(uint16_t addr, uint32_t data) {
  uint32_t cw = buildControlWord(false, addr);

  for (uint8_t attempt = 0; attempt < I2C_RETRIES; attempt++) {
    Wire1.beginTransmission(MCF_I2C_ADDR);

    // --- Control Word: MSB zuerst (Byte [23:16], [15:8], [7:0]) ---
    Wire1.write((uint8_t)((cw >> 16) & 0xFF));
    Wire1.write((uint8_t)((cw >>  8) & 0xFF));
    Wire1.write((uint8_t)( cw        & 0xFF));

    // --- Datenbytes: LSB zuerst (DB0, DB1, DB2, DB3) ---
    // Datasheet 7.6.2.2 / 7.6.2.4: "LSB byte is sent first"
    Wire1.write((uint8_t)( data        & 0xFF));
    Wire1.write((uint8_t)((data >>  8) & 0xFF));
    Wire1.write((uint8_t)((data >> 16) & 0xFF));
    Wire1.write((uint8_t)((data >> 24) & 0xFF));

    uint8_t err = Wire1.endTransmission(true);  // STOP senden

    if (err == 0) {
      return true;  // Erfolg
    }

    // NACK empfangen → Retry (lt. Datasheet: 5 Versuche empfohlen)
    Serial.printf("  [WARN] I2C-Write NACK, Versuch %d/%d, Addr=0x%03X, Err=%d\n",
                  attempt + 1, I2C_RETRIES, addr, err);
    delay(RETRY_DELAY_MS);
  }

  Serial.printf("[ERR]  I2C-Write endgültig fehlgeschlagen! Addr=0x%03X\n", addr);
  return false;
}

// ─────────────────────────────────────────────────────────────
//  I2C: 32-Bit Lesen
//  Sequenz (Datasheet 7.6.2.3):
//    START → [TARGET_ID + W=0] → ACK
//    → CW[23:16] → ACK → CW[15:8] → ACK → CW[7:0] → ACK
//    → REPEATED START → [TARGET_ID + R=1] → ACK
//    → DB0(LSB) → ACK → DB1 → ACK → DB2 → ACK → DB3(MSB) → NACK
//    → STOP
// ─────────────────────────────────────────────────────────────
static bool i2cRead32(uint16_t addr, uint32_t &result) {
  // Control Word mit OP_R/W = 1 (Bit 23 gesetzt)
  uint32_t cw = buildControlWord(true, addr);

  // Phase 1: Control Word senden (Bus-Richtung: Write, R/W-Bit = 0 im Adress-Byte!)
  // Das OP_R/W=1 ist im Control-Word, NICHT im I2C-Adress-Byte.
  for (uint8_t attempt = 0; attempt < I2C_RETRIES; attempt++) {
    Wire1.beginTransmission(MCF_I2C_ADDR);
    Wire1.write((uint8_t)((cw >> 16) & 0xFF));
    Wire1.write((uint8_t)((cw >>  8) & 0xFF));
    Wire1.write((uint8_t)( cw        & 0xFF));
    uint8_t err = Wire1.endTransmission(false);  // KEIN STOP → Repeated Start!

    if (err == 0) break;

    if (attempt == I2C_RETRIES - 1) {
      Serial.printf("[ERR]  I2C-Read (CW-Phase) fehlgeschlagen! Addr=0x%03X, Err=%d\n", addr, err);
      return false;
    }
    delay(RETRY_DELAY_MS);
  }

  // Phase 2: 4 Datenbytes lesen (Repeated Start → TARGET_ID + R=1)
  uint8_t received = Wire1.requestFrom((uint8_t)MCF_I2C_ADDR, (uint8_t)4, (uint8_t)true);
  if (received < 4) {
    Serial.printf("[ERR]  I2C-Read: Nur %d/4 Bytes empfangen! Addr=0x%03X\n", received, addr);
    return false;
  }

  // Daten rekonstruieren: LSB zuerst empfangen
  uint32_t b0 = Wire1.read();  // Byte 0 (LSB)
  uint32_t b1 = Wire1.read();  // Byte 1
  uint32_t b2 = Wire1.read();  // Byte 2
  uint32_t b3 = Wire1.read();  // Byte 3 (MSB)
  result = (b3 << 24) | (b2 << 16) | (b1 << 8) | b0;
  return true;
}

// ─────────────────────────────────────────────────────────────
//  MCF8316C-Q1 Initialisierung
// ─────────────────────────────────────────────────────────────
static bool mcf8316Init() {
  Serial.println("[MCF8316] Starte I2C-Initialisierung ...");

  // Warten bis der Chip nach Power-On bereit ist
  // Datasheet: DVDD muss stabil sein, Buck/LDO müssen hochgefahren sein
  delay(150);

  // ── Schritt 1: PIN_CONFIG1 setzen ──────────────────────────
  // Register-Adresse: 0x0A4 (EEPROM-Shadow)
  // Wert: 0x00000001  → SPEED_MODE = 01b (PWM)
  //
  // Hinweis: Schreiben in den Shadow-Register wirkt sofort im RAM.
  // Für dauerhafte Speicherung (nach Power-Cycle) wäre zusätzlich
  // der EEPROM-Brennvorgang nötig (REG_EEPROM_CTRL, EEPROM_WRITE_KEY).
  // Das wird hier NICHT gemacht um unnötige EEPROM-Zyklen zu vermeiden!
  Serial.print("[MCF8316] PIN_CONFIG1: SPEED_MODE = 01b (PWM) ... ");
  if (!i2cWrite32(REG_PIN_CONFIG1, PIN_CONFIG1_PWM_MODE)) {
    Serial.println("FEHLER!");
    return false;
  }
  Serial.println("OK");
  delay(10);

  // ── Schritt 2: Fault-Status lesen und ausgeben ─────────────
  uint32_t gdFault = 0, ctFault = 0;

  Serial.print("[MCF8316] Gate Driver Fault Status (0x0E0) ... ");
  if (i2cRead32(REG_GD_FAULT_STATUS, gdFault)) {
    Serial.printf("0x%08X  %s\n", gdFault,
                  (gdFault == 0) ? "→ kein Fehler ✓" : "→ FAULT AKTIV!");
    if (gdFault != 0) {
      // Bit 31: DRIVER_FAULT (gesamt)
      // Bit 28: OCP, Bit 26: OVP, Bit 25: OT (Overtemperature)
      if (gdFault & (1UL << 28)) Serial.println("        → OCP (Überstrom) erkannt");
      if (gdFault & (1UL << 26)) Serial.println("        → OVP (Überspannung) erkannt");
      if (gdFault & (1UL << 25)) Serial.println("        → OT  (Übertemperatur) erkannt");
    }
  } else {
    Serial.println("Lesefehler");
  }

  Serial.print("[MCF8316] Controller Fault Status  (0x0E2) ... ");
  if (i2cRead32(REG_CT_FAULT_STATUS, ctFault)) {
    Serial.printf("0x%08X  %s\n", ctFault,
                  (ctFault == 0) ? "→ kein Fehler ✓" : "→ FAULT AKTIV!");
  } else {
    Serial.println("Lesefehler");
  }

  // Bei aktiven Faults warnen, aber nicht abbrechen (könnte Lock-Detection sein)
  if (gdFault != 0 || ctFault != 0) {
    Serial.println("[WARN]  Aktive Faults erkannt. Prüfe Motor-Anschluss und Versorgung.");
    Serial.println("[WARN]  Faults werden nach erstem PWM-Signal ggf. automatisch gelöscht.");
  }

  return true;
}

// ─────────────────────────────────────────────────────────────
//  Motor-Steuerung
// ─────────────────────────────────────────────────────────────

// speed_pct: 0 = Stop, 100 = Vollgas
static void motorSetSpeed(uint8_t speed_pct) {
  if (speed_pct > 100) speed_pct = 100;
  // Teensy analogWrite: 8-Bit Auflösung (0–255)
  uint32_t pwmVal = ((uint32_t)speed_pct * 255UL) / 100UL;
  analogWrite(PWM_PIN, (int)pwmVal);
  Serial.printf("[Motor] Geschwindigkeit: %3d %%  (PWM = %3lu / 255)\n", speed_pct, pwmVal);
}

static void motorStop() {
  analogWrite(PWM_PIN, 0);
  Serial.println("[Motor] STOP — PWM = 0");
}

// ─────────────────────────────────────────────────────────────
//  Setup
// ─────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 9000);

  Serial.println("\n╔══════════════════════════════════════════╗");
  Serial.println("║  MCF8316C-Q1  BLDC Test  |  Teensy 4.0  ║");
  Serial.println("╚══════════════════════════════════════════╝\n");

  // ── PWM konfigurieren ──────────────────────────────────────
  // Teensy 4.0: analogWriteFrequency() setzt die Frequenz pro Pin
  // analogWriteResolution() setzt global die Bit-Tiefe
  pinMode(PWM_PIN, OUTPUT);
  analogWriteFrequency(PWM_PIN, PWM_FREQ_HZ);  // 20 kHz
  analogWriteResolution(8);                      // 8-Bit (0–255)
  motorStop();  // Sicher starten: PWM = 0
  Serial.printf("[PWM]   Pin %d, Frequenz = %lu Hz, Auflösung = 8-Bit\n",
                PWM_PIN, (unsigned long)PWM_FREQ_HZ);

  // ── I2C Wire1 initialisieren ───────────────────────────────
  // Teensy 4.0: Wire1 → SDA1 = Pin 17, SCL1 = Pin 16
  // 100 kHz Standard-Mode (MCF8316 unterstützt bis 400 kHz Fast-Mode,
  // aber 100 kHz gibt dem Chip mehr Reaktionszeit zwischen Bytes)
  Wire1.begin();
  Wire1.setClock(I2C_CLOCK_HZ);
  // Wire1.setDefaultTimeout(4000);  // optional: 4 ms Timeout (= MCF8316 Clock-Stretch-Limit)
  Serial.printf("[I2C]   Wire1 gestartet, Takt = %lu Hz\n", (unsigned long)I2C_CLOCK_HZ);
  Serial.printf("[I2C]   TARGET_ID = 0x%02X\n\n", MCF_I2C_ADDR);

  // ── MCF8316 initialisieren ─────────────────────────────────
  if (!mcf8316Init()) {
    Serial.println("\n[HALT]  Initialisierung fehlgeschlagen!");
    Serial.println("[HINT]  Prüfe: I2C-Pull-ups (4,7 kΩ nach 3,3 V)?");
    Serial.println("[HINT]  Prüfe: MCF8316 Versorgungsspannung (VVM)?");
    Serial.println("[HINT]  Prüfe: Verdrahtung SDA/SCL (Wire1: Pin 17/16)?");
    while (true) { delay(500); }
  }

  Serial.println("\n[OK]    Initialisierung abgeschlossen.");
  Serial.println("[INFO]  Testschleife: 10 s Motor EIN (50 %)  →  10 s Motor AUS\n");
}

// ─────────────────────────────────────────────────────────────
//  Hauptschleife
// ─────────────────────────────────────────────────────────────
void loop() {
  // Testgeschwindigkeit (anpassen nach Bedarf: 0–100)
  constexpr uint8_t TEST_SPEED_PCT = 50;
  constexpr uint32_t ON_DURATION_S  = 10;
  constexpr uint32_t OFF_DURATION_S = 10;

  // ─ Motor EIN ─────────────────────────────────────────────
  Serial.println("┌─────────────────────────────────────────┐");
  Serial.println("│            ▶  MOTOR EIN                 │");
  Serial.println("└─────────────────────────────────────────┘");
  motorSetSpeed(TEST_SPEED_PCT);

  for (uint32_t i = ON_DURATION_S; i > 0; i--) {
    Serial.printf("  Motor läuft  ...  noch %2lu Sekunden\n", i);
    delay(1000);
  }

  // ─ Motor AUS ─────────────────────────────────────────────
  Serial.println("┌─────────────────────────────────────────┐");
  Serial.println("│            ■  MOTOR AUS                 │");
  Serial.println("└─────────────────────────────────────────┘");
  motorStop();

  for (uint32_t i = OFF_DURATION_S; i > 0; i--) {
    Serial.printf("  Motor gestoppt  ...  noch %2lu Sekunden\n", i);
    delay(1000);
  }
}