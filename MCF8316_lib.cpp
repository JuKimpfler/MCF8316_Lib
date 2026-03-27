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
 *  Steuerprinzip (Strommodus / Torque-Mode):
 *    SPEED_MODE   = 01b  → PWM-Duty-Cycle auf Pin 33 steuert Sollstrom
 *    SPEED_LOOP_DIS = 1  → Strommodus (Torque-Mode) statt Drehzahlregelung
 *      Vorteil: funktioniert ohne motorbezogene Parameter (MAX_SPEED etc.)
 *      CURRENT_REF = Duty% × ILIMIT  (z.B. 50 % × 3 A = 1,5 A)
 *    → 0 % Duty  = Motor aus / Standby
 *    → 100 % Duty = ILIMIT (3 A) Strom-Sollwert
 *
 *  Warum NICHT reine Drehzahlregelung (SPEED_LOOP_DIS = 0)?
 *    Das EEPROM des Chips ist im Auslieferungszustand auf 0 gesetzt.
 *    MAX_SPEED = 0 → SPEED_REF = Duty × 0 = 0 → Motor bleibt immer im Standby.
 *    Für echte Drehzahlregelung muss MAX_SPEED (CLOSED_LOOP4) auf den
 *    motorbezogenen Wert in elektrischen Hz konfiguriert werden.
 *
 *  Testschleife:
 *    10 Sekunden Motor EIN (50 % → 1,5 A Sollstrom)  →  10 s Motor AUS  →  ...
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
constexpr uint32_t WAKE_PULSE_MS  = 30;       // SPEED/WAKE kurz statisch HIGH zum Aufwecken
constexpr uint32_t I2C_CLOCK_HZ   = 100000;   // 100 kHz  (Standard Mode)
                                               // Clock-Stretching deckt 100µs-Delay ab
constexpr uint8_t  I2C_RETRIES    = 5;        // Anzahl NACK-Retries (lt. Datasheet)
constexpr uint32_t RETRY_DELAY_MS = 5;        // Wartezeit zwischen Retries

static bool s_motorRunning = false;

// ─────────────────────────────────────────────────────────────
//  Register-Adressen  (MEM_ADDR-Teil, 12-Bit)
//  Vollständige Adresse: 0x000000 + Offset
//  Schreiben in Shadow-Register (0x80–0xAE) wirkt sofort im RAM;
//  kein EEPROM-Brennvorgang nötig für Laufzeit-Konfiguration.
// ─────────────────────────────────────────────────────────────

//  EEPROM Shadow: Motorstart-Konfiguration 1  (Datasheet 7.7.1.3, Offset 84h)
constexpr uint16_t REG_MOTOR_STARTUP1   = 0x084;

//  EEPROM Shadow: Motorstart-Konfiguration 2  (Datasheet 7.7.1.4, Offset 86h)
constexpr uint16_t REG_MOTOR_STARTUP2   = 0x086;

//  EEPROM Shadow: Closed-Loop-Konfiguration 1 (Datasheet 7.7.1.5, Offset 88h)
constexpr uint16_t REG_CLOSED_LOOP1     = 0x088;

//  EEPROM Shadow: Fehler-Konfiguration 1      (Datasheet 7.7.2.1, Offset 90h)
constexpr uint16_t REG_FAULT_CONFIG1    = 0x090;

//  EEPROM Shadow: Fehler-Konfiguration 2      (Datasheet 7.7.2.2, Offset 92h)
constexpr uint16_t REG_FAULT_CONFIG2    = 0x092;

//  EEPROM Shadow: Pin-Konfiguration 1         (Datasheet 7.7.3.1, Offset A4h)
//  Enthält SPEED_MODE – wirkt sofort auf den Shadow-RAM
constexpr uint16_t REG_PIN_CONFIG1      = 0x0A4;

//  Algorithm Control 1 / EEPROM-Trigger       (Datasheet 7.8.3.1, Offset EAh)
//  Enthält CLR_FLT (Bit 29), EEPROM_READ (Bit 30), EEPROM_WRT (Bit 31)
constexpr uint16_t REG_ALGO_CTRL1       = 0x0EA;

//  RAM – Fault-Status Register  (Datasheet 7.8.1, Offset E0h / E2h)
constexpr uint16_t REG_GD_FAULT_STATUS  = 0x0E0;  // Gate Driver Faults
constexpr uint16_t REG_CT_FAULT_STATUS  = 0x0E2;  // Controller Faults

// ─────────────────────────────────────────────────────────────
//  Registerwerte
// ─────────────────────────────────────────────────────────────

// ── MOTOR_STARTUP1 (0x084) ────────────────────────────────────
// Konfiguriert die Anlaufphase (Align-Modus, Standard-Startup).
//   Bits [28:25]  ALIGN_SLOW_RAMP_RATE = 7  → 100 A/s Stromanstieg
//   Bits [24:21]  ALIGN_TIME           = 5  → 400 ms Ausrichtungszeit
//   Bits [20:17]  ALIGN_OR_SLOW_CURRENT_ILIMIT = 3 → 1,0 A Ausrichtstrom
// Alle anderen Bits = 0 (Default: MTR_STARTUP = 00b = Align-Modus)
//
// Hinweis zum Default-Problem: ALIGN_SLOW_RAMP_RATE = 0 (0,1 A/s) + ALIGN_TIME = 0 (10 ms)
// ergibt im Auslieferungszustand nur ca. 0,001 A effektiv → Motor richtet sich nicht aus!
// (0,1 A/s × 10 ms = 0,001 A – bei weitem nicht genug zur Ausrichtung)
constexpr uint32_t MOTOR_STARTUP1_CFG =
    (7UL << 25) |   // ALIGN_SLOW_RAMP_RATE = 7 (100 A/s)
    (5UL << 21) |   // ALIGN_TIME           = 5 (400 ms)
    (3UL << 17);    // ALIGN_ILIMIT         = 3 (1,0 A)
                    // = 0x0EA60000

// ── MOTOR_STARTUP2 (0x086) ────────────────────────────────────
// Open-Loop-Beschleunigung und automatischer Wechsel zu Closed-Loop.
//   Bits [30:27]  OL_ILIMIT         = 5 → 2,0 A Stromgrenze im Open-Loop
//   Bits [26:23]  OL_ACC_A1         = 5 → 10 Hz/s lineare Beschleunigung
//   Bits [22:19]  OL_ACC_A2         = 4 →  5 Hz/s² quadratische Beschleunigung
//   Bit  [18]     AUTO_HANDOFF_EN   = 1 → automatischer OL→CL Übergang (BEMF-basiert)
//   Bits [2:0]    THETA_ERROR_RAMP_RATE = 5 → 0,5 °/ms Winkelkorrektur-Rate
//
// AUTO_HANDOFF_EN = 1 eliminiert die Abhängigkeit von MAX_SPEED beim OL→CL-Übergang.
constexpr uint32_t MOTOR_STARTUP2_CFG =
    (5UL << 27) |   // OL_ILIMIT             = 5 (2,0 A)
    (5UL << 23) |   // OL_ACC_A1             = 5 (10 Hz/s)
    (4UL << 19) |   // OL_ACC_A2             = 4 (5 Hz/s²)
    (1UL << 18) |   // AUTO_HANDOFF_EN       = 1
    5UL;            // THETA_ERROR_RAMP_RATE = 5 (0,5 °/ms)
                    // = 0x2AA40005

// ── CLOSED_LOOP1 (0x088) ─────────────────────────────────────
// Strommodus (Torque-Mode) aktivieren.
//   Bit [1]  SPEED_LOOP_DIS = 1 → Drehzahlregelung deaktiviert (Strommodus)
//
// Warum Strommodus? Im Default-EEPROM ist MAX_SPEED = 0.
// Im Drehzahlmodus (SPEED_LOOP_DIS = 0) gilt: SPEED_REF = Duty × MAX_SPEED = 0.
// Der Motor bleibt dauerhaft im Standby, obwohl PWM anliegt.
// Im Strommodus gilt: CURRENT_REF = Duty × ILIMIT (z.B. 50 % × 3 A = 1,5 A).
// Alle anderen Bits = 0 (Default: 10 kHz Ausgangs-PWM, CSVM-Modulation usw.)
constexpr uint32_t CLOSED_LOOP1_CFG =
    (1UL << 1);     // SPEED_LOOP_DIS = 1 (Strommodus)
                    // = 0x00000002

// ── FAULT_CONFIG1 (0x090) ────────────────────────────────────
// Stromgrenzen erhöhen; Software-Strombegrenzer-Lock deaktivieren.
//   Bits [30:27]  ILIMIT           = 7 → 3,0 A Closed-Loop Strom-Sollwert-Grenze
//   Bits [26:23]  HW_LOCK_ILIMIT   = 7 → 3,0 A HW-Komparator-Schutzschwelle
//   Bits [22:19]  LOCK_ILIMIT      = 7 → 3,0 A ADC-Sperrstrom-Schwelle
//   Bits [18:15]  LOCK_ILIMIT_MODE = 9 → ILIMIT-Lock-Erkennung deaktiviert
//
// Hinweis: Default HW_LOCK_ILIMIT = 0,125 A würde im Strombetrieb sofort auslösen!
constexpr uint32_t FAULT_CONFIG1_CFG =
    (7UL << 27) |   // ILIMIT         = 7 (3,0 A)
    (7UL << 23) |   // HW_LOCK_ILIMIT = 7 (3,0 A)
    (7UL << 19) |   // LOCK_ILIMIT    = 7 (3,0 A)
    (9UL << 15);    // LOCK_ILIMIT_MODE = 9 (deaktiviert)
                    // = 0x3BBC8000

// ── FAULT_CONFIG2 (0x092) ────────────────────────────────────
// Hardware-Komparator-Lock-Erkennung deaktivieren.
//   Bits [18:15]  HW_LOCK_ILIMIT_MODE = 9 → HW-Lock deaktiviert
//
// Hinweis: Default HW_LOCK_ILIMIT_MODE = 0 (Latched Fault, keine Entprellung)
// würde sofort bei Strom > 0,125 A auslösen.
constexpr uint32_t FAULT_CONFIG2_CFG =
    (9UL << 15);    // HW_LOCK_ILIMIT_MODE = 9 (deaktiviert)
                    // = 0x00048000

// ── PIN_CONFIG1 (0x0A4) ──────────────────────────────────────
// Bits [1:0]  SPEED_MODE = 01b  → PWM-Duty-Cycle auf SPEED-Pin steuert Motor
constexpr uint32_t PIN_CONFIG1_PWM_MODE = 0x00000001;

// ── ALGO_CTRL1 Steuerbits (0x0EA) ────────────────────────────
// Bit [29]  CLR_FLT = 1   → Alle latch-basierten Fehler löschen
// Bit [30]  EEPROM_READ   → EEPROM→Shadow laden (nur bei Bedarf)
// Bit [31]  EEPROM_WRT    → Shadow→EEPROM schreiben (nur bei Bedarf)
constexpr uint32_t ALGO_CTRL1_CLR_FLT   = (1UL << 29);  // = 0x20000000
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
//
//  Reihenfolge entspricht Datasheet 7.6.1.1 (EEPROM Write Procedure),
//  aber ohne abschließenden EEPROM-Brennvorgang – Änderungen wirken
//  nur für diesen Power-Cycle im Shadow-RAM.
// ─────────────────────────────────────────────────────────────
static bool mcf8316Init() {
  Serial.println("[MCF8316] Starte I2C-Initialisierung ...");

  // Warten bis der Chip nach Power-On bereit ist.
  // Datasheet: DVDD muss stabil sein, Buck/LDO müssen hochgefahren sein.
  delay(150);

  // ── Schritt 1: MOTOR_STARTUP1 – Anlaufparameter ────────────
  // Behebt das Default-Problem: 0,1 A/s Rampe + 10 ms Align-Zeit
  // ergibt im Auslieferungszustand praktisch 0 A → Motor richtet sich nicht aus.
  // Neu: 100 A/s Rampe, 400 ms Zeit, 1,0 A Strom → zuverlässige Ausrichtung.
  Serial.print("[MCF8316] MOTOR_STARTUP1 (0x084): Align 100A/s, 400ms, 1A ... ");
  if (!i2cWrite32(REG_MOTOR_STARTUP1, MOTOR_STARTUP1_CFG)) {
    Serial.println("FEHLER!");
    return false;
  }
  Serial.printf("OK (0x%08lX)\n", (unsigned long)MOTOR_STARTUP1_CFG);
  delay(10);

  // ── Schritt 2: MOTOR_STARTUP2 – Open-Loop-Parameter ────────
  // Konfiguriert den Open-Loop-Betrieb: 2 A Stromgrenze,
  // 10 Hz/s Beschleunigung, automatischer Übergang zu Closed-Loop
  // sobald BEMF erkannt wird (AUTO_HANDOFF_EN=1).
  Serial.print("[MCF8316] MOTOR_STARTUP2 (0x086): OL 2A, 10Hz/s, Auto-Handoff ... ");
  if (!i2cWrite32(REG_MOTOR_STARTUP2, MOTOR_STARTUP2_CFG)) {
    Serial.println("FEHLER!");
    return false;
  }
  Serial.printf("OK (0x%08lX)\n", (unsigned long)MOTOR_STARTUP2_CFG);
  delay(10);

  // ── Schritt 3: CLOSED_LOOP1 – Strommodus aktivieren ────────
  // SPEED_LOOP_DIS = 1: Betrieb im Strommodus (Torque-Mode).
  // Vermeidet das MAX_SPEED=0 Problem im Default-EEPROM, bei dem
  // SPEED_REF = Duty × 0 = 0 wäre → Motor bleibt immer im Standby.
  // Im Strommodus: CURRENT_REF = Duty% × ILIMIT (50% × 3A = 1,5A).
  Serial.print("[MCF8316] CLOSED_LOOP1 (0x088): Strommodus (SPEED_LOOP_DIS=1) ... ");
  if (!i2cWrite32(REG_CLOSED_LOOP1, CLOSED_LOOP1_CFG)) {
    Serial.println("FEHLER!");
    return false;
  }
  Serial.printf("OK (0x%08lX)\n", (unsigned long)CLOSED_LOOP1_CFG);
  delay(10);

  // ── Schritt 4: FAULT_CONFIG1 – Stromgrenzen erhöhen ────────
  // Default ILIMIT / HW_LOCK_ILIMIT = 0,125 A würde im Strombetrieb
  // sofort einen Latched-Fault auslösen.
  // Neu: ILIMIT = 3 A, Lock-Erkennung deaktiviert.
  Serial.print("[MCF8316] FAULT_CONFIG1 (0x090): ILIMIT=3A, Lock disab. ... ");
  if (!i2cWrite32(REG_FAULT_CONFIG1, FAULT_CONFIG1_CFG)) {
    Serial.println("FEHLER!");
    return false;
  }
  Serial.printf("OK (0x%08lX)\n", (unsigned long)FAULT_CONFIG1_CFG);
  delay(10);

  // ── Schritt 5: FAULT_CONFIG2 – HW-Lock deaktivieren ────────
  // Default HW_LOCK_ILIMIT_MODE = 0 (Latched Fault, kein Deglitch)
  // würde sofort bei Strom > 0,125 A auslösen.
  Serial.print("[MCF8316] FAULT_CONFIG2 (0x092): HW-Lock deaktiviert ... ");
  if (!i2cWrite32(REG_FAULT_CONFIG2, FAULT_CONFIG2_CFG)) {
    Serial.println("FEHLER!");
    return false;
  }
  Serial.printf("OK (0x%08lX)\n", (unsigned long)FAULT_CONFIG2_CFG);
  delay(10);

  // ── Schritt 6: PIN_CONFIG1 – SPEED_MODE = 01b (PWM) ────────
  // Konfiguriert die Geschwindigkeits-Eingabequelle auf PWM-Duty-Cycle.
  // Hinweis: Schreiben in den Shadow-Register wirkt sofort im RAM.
  // Für dauerhafte Speicherung (nach Power-Cycle) wäre zusätzlich
  // der EEPROM-Brennvorgang nötig (REG_ALGO_CTRL1, EEPROM_WRITE_KEY).
  // Das wird hier NICHT gemacht, um unnötige EEPROM-Zyklen zu vermeiden!
  Serial.print("[MCF8316] PIN_CONFIG1  (0x0A4): SPEED_MODE = 01b (PWM) ... ");
  if (!i2cWrite32(REG_PIN_CONFIG1, PIN_CONFIG1_PWM_MODE)) {
    Serial.println("FEHLER!");
    return false;
  }
  Serial.printf("OK (0x%08lX)\n", (unsigned long)PIN_CONFIG1_PWM_MODE);
  delay(10);

  // ── Schritt 7: Fault-Status lesen und ausgeben ─────────────
  uint32_t gdFault = 0, ctFault = 0;

  Serial.print("[MCF8316] Gate Driver Fault Status (0x0E0) ... ");
  if (i2cRead32(REG_GD_FAULT_STATUS, gdFault)) {
    Serial.printf("0x%08lX  %s\n", (unsigned long)gdFault,
                  (gdFault == 0) ? "kein Fehler" : "FAULT AKTIV!");
    if (gdFault != 0) {
      if (gdFault & (1UL << 28)) Serial.println("        → OCP (Überstrom) erkannt");
      if (gdFault & (1UL << 26)) Serial.println("        → OVP (Überspannung) erkannt");
      if (gdFault & (1UL << 25)) Serial.println("        → OT  (Übertemperatur) erkannt");
    }
  } else {
    Serial.println("Lesefehler");
  }

  Serial.print("[MCF8316] Controller Fault Status  (0x0E2) ... ");
  if (i2cRead32(REG_CT_FAULT_STATUS, ctFault)) {
    Serial.printf("0x%08lX  %s\n", (unsigned long)ctFault,
                  (ctFault == 0) ? "kein Fehler" : "FAULT AKTIV!");
  } else {
    Serial.println("Lesefehler");
  }

  // ── Schritt 8: CLR_FLT – Latch-Fehler löschen ─────────────
  // Latched Faults (z.B. von vorherigen Betriebsphasen) blockieren den Motor.
  // CLR_FLT (Bit 29 in ALGO_CTRL1) setzt alle Fehler-Latch-Bits zurück.
  // Datasheet 7.4.1.3: bis zu 200 ms nach CLR_FLT bis Fehler gelöscht sind.
  if (gdFault != 0 || ctFault != 0) {
    Serial.println("[MCF8316] Aktive Faults erkannt – sende CLR_FLT ...");
  } else {
    Serial.println("[MCF8316] Sende CLR_FLT (präventiv) ...");
  }
  i2cWrite32(REG_ALGO_CTRL1, ALGO_CTRL1_CLR_FLT);
  delay(200);  // Datasheet: bis zu 200 ms Wartezeit nach CLR_FLT

  return true;
}

// ─────────────────────────────────────────────────────────────
//  Motor-Steuerung
// ─────────────────────────────────────────────────────────────

// speed_pct: 0 = Stop, 100 = Vollgas
static void motorSetSpeed(uint8_t speed_pct) {
  if (speed_pct > 100) speed_pct = 100;
  const bool shouldSendWakePulse = (speed_pct > 0 && !s_motorRunning);

  // SPEED/WAKE-Pin braucht nach Standby einen stabilen HIGH-Pegel.
  // Reines 20-kHz-PWM kann als Wake-Signal vom Eingang nicht sicher erkannt werden.
  if (shouldSendWakePulse) {
    digitalWrite(PWM_PIN, HIGH);
    delay(WAKE_PULSE_MS);  // intentionally blocking: one-time short wake pulse at start
  }

  // Teensy analogWrite: 8-Bit Auflösung (0–255)
  uint32_t pwmVal = ((uint32_t)speed_pct * 255UL) / 100UL;
  analogWrite(PWM_PIN, (int)pwmVal);
  if (shouldSendWakePulse) {
    Serial.printf("[Motor] Geschwindigkeit: %3d %%  (PWM = %3lu / 255, Wake=%lums)\n",
                  speed_pct, pwmVal, (unsigned long)WAKE_PULSE_MS);
  } else {
    Serial.printf("[Motor] Geschwindigkeit: %3d %%  (PWM = %3lu / 255)\n", speed_pct, pwmVal);
  }
  s_motorRunning = (speed_pct > 0);
}

static void motorStop() {
  analogWrite(PWM_PIN, 0);
  s_motorRunning = false;
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
  Serial.println("[INFO]  Strommodus aktiv: CURRENT_REF = Duty% × 3A");
  Serial.println("[INFO]  Testschleife: 10 s Motor EIN (50 % → 1,5 A)  →  10 s Motor AUS\n");
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