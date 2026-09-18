/**
 * @file example_04_rs485_modbus.ino
 * @brief HiveNode Example 04 — RS485 Modbus RTU master via UART multiplexer
 *
 * Polls a Modbus RTU slave (temperature/humidity/version registers)
 * over RS485, sharing RAK3172's Serial1 with GPS and Bluetooth
 * through the TS3A5017DR multiplexer. See README.md for the full
 * multiplexer channel map.
 *
 * KEY LESSONS LEARNED:
 *
 * 1. RS485 is a half-duplex bus: the DIR pin (tied to both DE and
 *    RE# on the transceiver) must be HIGH only while transmitting,
 *    then LOW to listen. Dropping DIR too early — right after the
 *    last byte is queued instead of after it has actually finished
 *    shifting out on the wire — truncates the last byte on the bus.
 *    This code waits a fixed per-byte time (BYTE_TIME_US, derived
 *    from the 9600 baud / 10 bits-per-byte frame) after every
 *    Serial1.write() call, plus a 3.5-character Modbus inter-frame
 *    gap before switching back to receive.
 *
 * 2. RS485 is on multiplexer channel IN2=L, IN1=H (S2) — different
 *    from GPS (IN2=H, IN1=L) and Bluetooth (IN2=L, IN1=L). Always
 *    confirm this by looping the A/B lines together temporarily and
 *    checking the sent bytes echo back exactly, rather than trusting
 *    the datasheet's channel numbering by inspection.
 *
 * 3. Serial1 runs at 9600 baud here, same as GPS but different from
 *    Bluetooth's 115200 — if you switch the multiplexer between
 *    RS485/GPS and Bluetooth in the same firmware, Serial1 must be
 *    closed and reopened at the correct baud rate for whichever
 *    device is currently selected.
 *
 * 4. This is a minimal Modbus RTU master (function 0x03, Read
 *    Holding Registers) with CRC16 (Modbus polynomial 0xA001).
 *    Expand modbusReadRegs()/parseModbus() for other function codes
 *    or a specific slave's register map.
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
#define DIR_PIN PB5   // RS485 transceiver DE+RE (tied together)

// 9600 baud, 10 bits per byte (start + 8 data + stop) = 1.0417ms/byte
#define BYTE_TIME_US 1042

uint16_t crc16(uint8_t *buf, uint8_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= buf[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 1) crc = (crc >> 1) ^ 0xA001;
            else         crc >>= 1;
        }
    }
    return crc;
}

uint8_t  rxBuf[32];
uint8_t  rxLen    = 0;
uint32_t lastByte = 0;
bool     waiting  = false;
uint32_t sendTime = 0;

// RS485 is on IN2=LOW, IN1=HIGH (confirmed by looping A/B and
// checking the sent bytes echo back exactly)
void switchToRS485()
{
    digitalWrite(MUX_EN,  LOW);
    digitalWrite(MUX_IN2, LOW);
    digitalWrite(MUX_IN1, HIGH);
}

void modbusSend(uint8_t *buf, uint8_t len)
{
    uint16_t crc = crc16(buf, len);
    buf[len]   = crc & 0xFF;
    buf[len + 1] = crc >> 8;
    len += 2;

    while (Serial1.available()) Serial1.read();
    rxLen = 0;

    Serial.print("TX: ");
    for (int i = 0; i < len; i++) Serial.printf("%02X ", buf[i]);
    Serial.println();

    // Drive the bus, send each byte, and wait for it to actually
    // finish shifting out before sending the next one
    digitalWrite(DIR_PIN, HIGH);
    for (int i = 0; i < len; i++) {
        Serial1.write(buf[i]);
        delayMicroseconds(BYTE_TIME_US + 200); // +200us margin
    }
    // Modbus inter-frame gap: 3.5 character times
    delayMicroseconds(BYTE_TIME_US * 4);
    digitalWrite(DIR_PIN, LOW); // release the bus, listen for the reply

    waiting  = true;
    sendTime = millis();
}

void modbusReadRegs(uint8_t addr, uint16_t reg, uint16_t count)
{
    uint8_t buf[8];
    buf[0] = addr;
    buf[1] = 0x03; // Read Holding Registers
    buf[2] = reg >> 8;
    buf[3] = reg & 0xFF;
    buf[4] = count >> 8;
    buf[5] = count & 0xFF;
    modbusSend(buf, 6);
}

void parseModbus(uint8_t *buf, uint8_t len)
{
    if (len < 5) {
        Serial.printf("ERR: too short (%d bytes)\r\n", len);
        return;
    }

    uint16_t crc_calc = crc16(buf, len - 2);
    uint16_t crc_recv = buf[len - 2] | (buf[len - 1] << 8);
    if (crc_calc != crc_recv) {
        Serial.printf("ERR: CRC calc=%04X recv=%04X\r\n", crc_calc, crc_recv);
        return;
    }

    if (buf[0] != 0x01) {
        Serial.printf("ERR: wrong slave ID %02X\r\n", buf[0]);
        return;
    }

    uint8_t reg_count = buf[2] / 2;
    Serial.printf("OK: %d registers\r\n", reg_count);
    for (uint8_t i = 0; i < reg_count; i++) {
        uint16_t val = (buf[3 + i * 2] << 8) | buf[4 + i * 2];
        switch (i) {
            case 0: Serial.printf("  Temp: %.1f C\r\n",  val / 10.0); break;
            case 1: Serial.printf("  Hum:  %.1f %%\r\n", val / 10.0); break;
            case 2: Serial.printf("  Ver:  0x%04X\r\n",  val);        break;
            default: Serial.printf("  [%d]: %d\r\n", i, val);         break;
        }
    }
}

void setup()
{
    pinMode(LED, OUTPUT);
    pinMode(MUX_EN,  OUTPUT);
    pinMode(MUX_IN1, OUTPUT);
    pinMode(MUX_IN2, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    digitalWrite(DIR_PIN, LOW); // start in receive mode

    Serial.begin(115200);
    delay(2000);

    // Close and reopen Serial1 at the RS485 baud rate, in custom
    // mode so RUI3's AT-command parser does not intercept bytes
    Serial1.end();
    delay(100);
    Serial1.begin(9600, RAK_CUSTOM_MODE);
    delay(300);
    while (Serial1.available()) Serial1.read();

    switchToRS485();

    Serial.println("=== HiveNode Example 04 — RS485 Modbus RTU Master ===");
}

void loop()
{
    static uint32_t lastPoll = 0;
    static uint8_t  step     = 0;

    if (!waiting && millis() - lastPoll > 3000) {
        lastPoll = millis();
        switch (step) {
            case 0:
                Serial.println("\r\n--- Read TEMP ---");
                modbusReadRegs(1, 0x0000, 1);
                break;
            case 1:
                Serial.println("\r\n--- Read HUM ---");
                modbusReadRegs(1, 0x0001, 1);
                break;
            case 2:
                Serial.println("\r\n--- Read ALL ---");
                modbusReadRegs(1, 0x0000, 3);
                break;
        }
        step = (step + 1) % 3;
    }

    while (Serial1.available()) {
        uint8_t b = Serial1.read();
        if (rxLen < sizeof(rxBuf))
            rxBuf[rxLen++] = b;
        lastByte = millis();
    }

    if (rxLen > 0 && millis() - lastByte > 100) {
        Serial.print("RX: ");
        for (int i = 0; i < rxLen; i++) Serial.printf("%02X ", rxBuf[i]);
        Serial.println();
        parseModbus(rxBuf, rxLen);
        rxLen   = 0;
        waiting = false;
    }

    if (waiting && millis() - sendTime > 2000) {
        Serial.println("TIMEOUT");
        waiting = false;
        rxLen   = 0;
    }
}
