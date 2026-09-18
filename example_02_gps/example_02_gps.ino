/**
 * @file example_02_gps.ino
 * @brief HiveNode Example 02 — ATGM336H-5NR-32 GPS via UART multiplexer
 *
 * Reads NMEA sentences from the GPS module through the TS3A5017DR
 * analog multiplexer that shares RAK3172's Serial1 (UART1, PB6/PB7)
 * between three peripherals on this board: GPS, Bluetooth (E104-BT52),
 * and RS485. See README.md for the multiplexer channel map.
 *
 * KEY LESSONS LEARNED:
 *
 * 1. The multiplexer's EN pin is ACTIVE LOW (confirmed against the
 *    TS3A5017DR datasheet function table: EN=L selects a channel by
 *    IN2/IN1, EN=H turns everything off). This is opposite to what
 *    the pin name might suggest at a glance.
 *
 * 2. The IN2/IN1 -> S1..S4 truth table numbering in the datasheet
 *    (channels numbered from S1) does NOT match a naive 0-indexed
 *    reading of the two select bits — always verify the physical
 *    channel with a multimeter rather than trusting the datasheet
 *    table by inspection alone. On this board the confirmed mapping is:
 *      IN2=L, IN1=L -> Bluetooth (E104-BT52)
 *      IN2=L, IN1=H -> RS485
 *      IN2=H, IN1=L -> GPS   (used below)
 *      IN2=H, IN1=H -> unused
 *
 * 3. Serial1 must be fully closed and reopened with RAK_CUSTOM_MODE
 *    whenever the required baud rate changes (GPS/RS485 use 9600,
 *    Bluetooth uses 115200). RAK_CUSTOM_MODE also disables RUI3's
 *    own AT-command parser on that UART, which otherwise intercepts
 *    and swallows bytes meant for the external device.
 *
 * 4. The "ANTENNA OPEN" NMEA text message from the ATGM336H is a
 *    warning, not a fault: it appears because this board's AT2659S
 *    LNA + passive patch antenna setup doesn't match the current-
 *    sense check the GPS module expects for an "active" antenna.
 *    Fixes still work normally.
 *
 * 5. First fix (cold start) can take 1-15 minutes depending on sky
 *    view. Indoors near a window, expect a weak or absent fix;
 *    outdoors with open sky, typically ~1-2 minutes.
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

#include <TinyGPSPlus.h>

// ===== Pin definitions =====
#define LED     PA5
#define MUX_EN  PB2   // TS3A5017DR enable, ACTIVE LOW
#define MUX_IN1 PA15  // Select bit 1
#define MUX_IN2 PA10  // Select bit 2

TinyGPSPlus gps;

void blink(int times, int ms = 200)
{
    for (int i = 0; i < times; i++) {
        digitalWrite(LED, HIGH); delay(ms);
        digitalWrite(LED, LOW);  delay(ms);
    }
}

// GPS is on IN2=HIGH, IN1=LOW (confirmed with multimeter — see notes above)
void switchToGPS()
{
    digitalWrite(MUX_EN,  LOW);
    digitalWrite(MUX_IN2, HIGH);
    digitalWrite(MUX_IN1, LOW);
}

void setup()
{
    pinMode(LED, OUTPUT);
    pinMode(MUX_EN,  OUTPUT);
    pinMode(MUX_IN1, OUTPUT);
    pinMode(MUX_IN2, OUTPUT);

    Serial.begin(115200);
    delay(2000);

    // Close and reopen Serial1 at the GPS baud rate, in custom mode
    // so RUI3's AT-command parser does not intercept NMEA bytes
    Serial1.end();
    delay(100);
    Serial1.begin(9600, RAK_CUSTOM_MODE);
    delay(300);

    switchToGPS();

    blink(3);
    Serial.println("=== HiveNode Example 02 — GPS (ATGM336H-5NR-32) ===");
    Serial.println("Waiting for NMEA data / satellite fix...");
}

void loop()
{
    while (Serial1.available())
        gps.encode(Serial1.read());

    if (gps.location.isUpdated()) {
        Serial.printf("LAT=%.6f LON=%.6f ALT=%.1fm SAT=%d HDOP=%.1f\r\n",
            gps.location.lat(),
            gps.location.lng(),
            gps.altitude.meters(),
            gps.satellites.value(),
            gps.hdop.hdop());
    }
}
