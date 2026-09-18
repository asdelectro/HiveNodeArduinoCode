/**
 * @file example_03_bluetooth.ino
 * @brief HiveNode Example 03 — E104-BT52 BLE module via UART multiplexer
 *
 * Talks AT commands to the Ebyte E104-BT52 (DA14531-based BLE 5.0
 * module) through the same TS3A5017DR multiplexer used for GPS and
 * RS485 in example_02 and example_04. See README.md for the full
 * multiplexer channel map.
 *
 * KEY LESSONS LEARNED:
 *
 * 1. E104-BT52 runs its AT-command UART at 115200 baud, NOT 9600 —
 *    this was the very first thing that made "AT" appear to get no
 *    response at all when testing it at the wrong baud rate.
 *
 * 2. The module responds "+OK" to a bare "AT", and "+OK=<value>" to
 *    query commands like "AT+VER?" / "AT+NAME?". A response of a
 *    bare "OK" (no plus sign) or "AT_COMMAND_NOT_FOUND" is actually
 *    coming from RAK3172's own AT parser on Serial, NOT from the
 *    Bluetooth module — this happens if Serial1 is opened in the
 *    default mode instead of RAK_CUSTOM_MODE, since RUI3 quietly
 *    intercepts anything that looks like an AT command on any of
 *    its UARTs.
 *
 * 3. WKP (wake) needs a HIGH pulse, not a held HIGH level, to force
 *    the module into AT-command mode: pulse it HIGH for ~200us then
 *    back LOW. Simply holding WKP high the whole time did not make
 *    the module answer reliably in testing.
 *
 * 4. Bluetooth is on multiplexer channel IN2=L, IN1=L (S1). This
 *    was found by systematically cycling all 4 channel combinations
 *    and sending "AT" on each one, watching for a real "+OK" reply.
 *
 * Hardware: HiveNode v1.0 (RAK3172-T based)
 * More info: https://hivenode.net
 *
 * MIT License
 * Copyright (c) 2026 HiveNode / Aleksii
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND.
 */

// ===== Pin definitions =====
#define LED     PA5
#define MUX_EN  PB2   // TS3A5017DR enable, ACTIVE LOW
#define MUX_IN1 PA15  // Select bit 1
#define MUX_IN2 PA10  // Select bit 2
#define BT_WKP  PB3   // E104-BT52 wake / AT-mode pin

void blink(int times, int ms = 200)
{
    for (int i = 0; i < times; i++) {
        digitalWrite(LED, HIGH); delay(ms);
        digitalWrite(LED, LOW);  delay(ms);
    }
}

// Bluetooth is on IN2=LOW, IN1=LOW (confirmed by cycling all
// 4 channels and checking which one answers "AT" with "+OK")
void switchToBT()
{
    digitalWrite(MUX_EN,  LOW);
    digitalWrite(MUX_IN2, LOW);
    digitalWrite(MUX_IN1, LOW);
}

// A short HIGH pulse (not a held level) forces the module into
// AT-command mode reliably
void btWake()
{
    digitalWrite(BT_WKP, HIGH);
    delayMicroseconds(200);
    digitalWrite(BT_WKP, LOW);
    delay(300);
}

String btAT(const char *cmd, uint32_t timeout_ms = 1000)
{
    Serial.print(">> ");
    Serial.println(cmd);

    Serial1.print(cmd);

    String resp = "";
    uint32_t t = millis();
    while (millis() - t < timeout_ms) {
        while (Serial1.available())
            resp += (char)Serial1.read();
        if (resp.endsWith("\r\n"))
            break;
    }
    resp.trim();

    Serial.print("<< ");
    Serial.println(resp);
    return resp;
}

void setup()
{
    pinMode(LED, OUTPUT);
    pinMode(MUX_EN,  OUTPUT);
    pinMode(MUX_IN1, OUTPUT);
    pinMode(MUX_IN2, OUTPUT);
    pinMode(BT_WKP,  OUTPUT);
    digitalWrite(BT_WKP, LOW);

    Serial.begin(115200);
    delay(2000);

    // Close and reopen Serial1 at 115200 in custom mode, so RUI3's
    // own AT-command parser does not intercept bytes meant for the
    // Bluetooth module
    Serial1.end();
    delay(100);
    Serial1.begin(115200, RAK_CUSTOM_MODE);
    delay(300);

    switchToBT();
    delay(500);
    btWake();

    blink(3);
    Serial.println("=== HiveNode Example 03 — Bluetooth (E104-BT52) ===");

    btAT("AT");
    btAT("AT+VER?");
    btAT("AT+NAME?");
    btAT("AT+ADDR?");
}

void loop()
{
    // Bluetooth -> Serial Monitor
    if (Serial1.available())
        Serial.write(Serial1.read());

    // Serial Monitor -> Bluetooth (type AT commands directly)
    if (Serial.available()) {
        String line = Serial.readStringUntil('\n');
        line.trim();
        if (line.length() > 0)
            Serial1.print(line);
    }
}
