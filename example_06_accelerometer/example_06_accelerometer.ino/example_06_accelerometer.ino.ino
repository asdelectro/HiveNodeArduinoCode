/**
 * @file example_06_accelerometer.ino
 * @brief HiveNode Example 06 — LIS2DW12 accelerometer, wake-on-motion
 *
 * Direct I2C register access to the LIS2DW12 accelerometer
 * (no Adafruit/SparkFun/STMicroelectronics library) to configure
 * hardware wake-up-on-motion and poll the INT1 pin for activity.
 *
 * As with example_01's BQ25628 access, this is done at the raw
 * register level rather than through a library. In this project's
 * case the motivation is different from BQ25628 (which had a
 * library that outright failed to `begin()` on this chip revision):
 * here it is simply that hardware wake-on-motion via INT1/WAKE_SRC
 * needs direct control over CTRL3/CTRL4_INT1/CTRL7 and the wake
 * threshold/duration registers that most accelerometer libraries
 * don't expose cleanly for this exact use case.
 *
 * KEY LESSONS LEARNED:
 *
 * 1. WHO_AM_I (register 0x0F) reads back 0x44 for the LIS2DW12 —
 *    do not confuse this with the SHT40's I2C *address* (also
 *    0x44 on this board's I2C bus, but a different register/chip
 *    entirely). Always verify against WHO_AM_I before assuming a
 *    device is present, not just against the bus scan address.
 *
 * 2. CTRL7's INTERRUPTS_ENABLE bit (0x20) is mandatory for the
 *    wake-up interrupt to ever appear on INT1. Configuring
 *    CTRL3/CTRL4_INT1/WAKE_THS/WAKE_DUR correctly but forgetting
 *    this one register means INT1 simply never goes high, with no
 *    other error indication.
 *
 * 3. This example polls the INT1 pin directly in loop() rather
 *    than using attachInterrupt(). That's a deliberate choice for
 *    this Level 1 example, to keep the register-level behaviour
 *    visible and avoid interrupt-context complications — see
 *    example_01_sensors_sleep for how a timer-driven wake pattern
 *    is used instead once this moves to a board-level example.
 *
 * 4. Low-power mode here is CTRL1 = 0x21 (12.5 Hz, Low-Power
 *    Mode 1), which is enough for motion detection while keeping
 *    the accelerometer's own current draw low; it does not by
 *    itself sleep the MCU — see example_01 for the MCU-side sleep
 *    pattern if combining the two.
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

#include <Arduino.h>
#include <Wire.h>

// ===== Pin definitions =====
#define SENSOR_PWR  PA9   // Power rail for the I2C sensor bus
#define PWR2        PA8   // Second power rail used on this board
#define ACC_INT     PB4   // LIS2DW12 INT1, polled (not attachInterrupt)
#define LIS_ADDR    0x18  // I2C address (SA0 = 0)

// ===== LIS2DW12 registers used =====
#define CTRL1       0x20  // ODR, mode, low-power sub-mode
#define CTRL2       0x21  // BDU, auto-increment, soft reset
#define CTRL3       0x22  // Interrupt polarity/latching
#define CTRL4_INT1  0x23  // Routes wake-up event to INT1
#define CTRL6       0x25  // Full-scale range, filtering, low-noise
#define WAKE_THS    0x34  // Wake-up threshold
#define WAKE_DUR    0x35  // Wake-up duration
#define WAKE_SRC    0x38  // Wake-up event source (read to clear)
#define CTRL7       0x3F  // INTERRUPTS_ENABLE — required, see notes above

bool lisWrite(uint8_t reg, uint8_t val)
{
    Wire.beginTransmission(LIS_ADDR);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission() == 0;
}

bool lisRead(uint8_t reg, uint8_t *buf, uint8_t len)
{
    Wire.beginTransmission(LIS_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;

    Wire.requestFrom((uint8_t)LIS_ADDR, len);
    uint32_t t = millis();
    for (uint8_t i = 0; i < len; i++) {
        while (!Wire.available()) {
            if (millis() - t > 50) return false;
        }
        buf[i] = Wire.read();
    }
    return true;
}

void lisInit()
{
    lisWrite(CTRL2,      0x0C);  // BDU + IF_ADD_INC
    lisWrite(CTRL6,      0x04);  // ±2g, LOW_NOISE
    lisWrite(CTRL1,      0x21);  // ODR 12.5 Hz, Low-Power Mode 1
    delay(50);

    lisWrite(CTRL3,      0x00);  // pulsed, active HIGH
    lisWrite(WAKE_THS,   0x0A);  // threshold ~620 mg
    lisWrite(WAKE_DUR,   0x00);  // no extra delay
    lisWrite(CTRL4_INT1, 0x20);  // route wake-up event to INT1

    // Mandatory — without this bit set, INT1 never goes high no
    // matter how correctly everything else above is configured
    lisWrite(CTRL7,      0x20);
}

void setup()
{
    pinMode(SENSOR_PWR, OUTPUT); digitalWrite(SENSOR_PWR, HIGH);
    pinMode(PWR2,       OUTPUT); digitalWrite(PWR2,       HIGH);
    delay(100);

    Serial.begin(115200);
    delay(2000);
    Serial.println("=== HiveNode Example 06 — LIS2DW12 motion wake-up ===");

    Wire.begin();
    Wire.setClock(100000);
    delay(50);

    // Confirm the chip via WHO_AM_I before doing anything else —
    // this is a different check than the I2C bus-scan address
    uint8_t id;
    lisRead(0x0F, &id, 1);
    Serial.printf("WHO_AM_I: 0x%02X (expect 0x44)\r\n", id);
    if (id != 0x44) {
        Serial.println("ERROR: LIS2DW12 not found!");
        while (1) delay(1000);
    }

    pinMode(ACC_INT, INPUT);
    lisInit();
    Serial.println("Ready. Shake to trigger!");
}

void loop()
{
    // Polling the pin directly — no attachInterrupt() in this
    // Level 1 example, see notes above
    if (digitalRead(ACC_INT) == HIGH) {
        uint8_t src;
        lisRead(WAKE_SRC, &src, 1); // reading this register clears it

        Serial.println("*** MOTION DETECTED ***");
        Serial.printf("    WAKE_UP_SRC: 0x%02X\r\n", src);

        // Hook whatever action is needed here, e.g. enabling
        // Bluetooth (example_03) or sending a LoRaWAN alarm packet

        delay(500); // debounce
    }
}
