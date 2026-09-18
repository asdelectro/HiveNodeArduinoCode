/**
 * @file example_01_sensors_sleep.ino
 * @brief HiveNode Example 01 — SHT40 + BQ25628 + event-driven sleep
 *
 * Reads temperature/humidity from SHT40 and battery/charger status
 * from BQ25628 every 10 seconds using RUI3's periodic timer, then
 * returns to a deep sleep state (~18uA measured on oscilloscope).
 *
 * KEY LESSONS LEARNED (see README.md for the full story):
 *
 * 1. loop() must call setAllPinsAnalog() + api.system.sleep.all(),
 *    NOT setup(). The RUI3 scheduler re-enters loop() after every
 *    wake event (timer, interrupt, etc), so the sleep call has to
 *    live there — calling it once at the end of setup() means the
 *    device sleeps exactly once and then stays awake forever after
 *    the first wake.
 *
 * 2. sensor_handler() (called by the timer) must call Wire.begin()
 *    again every time it runs. setAllPinsAnalog() puts PA11/PA12
 *    (I2C SDA/SCL) into analog mode during sleep, so the I2C
 *    peripheral must be re-initialized on every wake or the sensor
 *    read will hang.
 *
 * 3. BQ25628's ADC must be explicitly turned OFF
 *    (write 0x00 to REG_ADC_CONTROL) at the end of every
 *    sensor_handler() call, right before going back to sleep.
 *    Left in continuous mode, the charger IC keeps converting on
 *    its own and pulled the whole board's sleep current from
 *    ~18uA up to ~580uA — this was the single biggest current
 *    leak found while building this example.
 *
 * 4. On USB disconnect, PA2/PA3 (UART2, used by an external UART
 *    multiplexer on other HiveNode boards) must be released to
 *    INPUT before rebooting, otherwise the multiplexer keeps
 *    VBUS_F energized and a power-indicator LED stays lit even
 *    though the board runs on battery only.
 *
 * NOTE: reading sensors every 10 seconds still produces a fairly
 * high *average* current, because each wake cycle briefly pulls
 * several mA while I2C is active. For real deployments, increase
 * the timer interval (e.g. 60-600s) to bring the average down —
 * the 18uA sleep floor is unaffected by this, only the duty cycle
 * of the active phase changes the average.
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

#include "stm32wlxx_hal.h"
#include <Wire.h>
#include <Adafruit_SHT4x.h>

// ===== Pin definitions =====
#define LED         PA5   // Status LED
#define SENSOR_PWR  PA9   // Power rail for SHT40 / BQ25628 I2C bus

// ===== BQ25628 I2C address and registers =====
#define BQ_ADDR             0x6A
#define REG_CHARGER_CTRL0   0x16  // Watchdog, EN_CHG, EN_HIZ
#define REG_CHARGER_CTRL3   0x19  // EN_EXTILIM
#define REG_ICHG            0x02  // Charge current limit
#define REG_ADC_CONTROL     0x26  // ADC enable/mode
#define REG_ADC_DISABLE     0x27  // ADC channel disable
#define REG_IBAT_ADC        0x2A  // IBAT ADC reading
#define REG_VBUS_ADC        0x2C  // VBUS ADC reading
#define REG_VBAT_ADC        0x30  // VBAT ADC reading
#define REG_VSYS_ADC        0x32  // VSYS ADC reading
#define REG_CHARGER_STATUS1 0x1E  // VBUS_STAT, CHG_STAT

Adafruit_SHT4x sht4x;
bool has_sht40   = false;
bool wasConnected = false;

// ===== Utility =====
void blink(int times, int ms = 200)
{
    for (int i = 0; i < times; i++) {
        digitalWrite(LED, HIGH); delay(ms);
        digitalWrite(LED, LOW);  delay(ms);
    }
}

// ===== BQ25628 I2C helpers =====
void writeReg(uint8_t reg, uint8_t val)
{
    Wire.beginTransmission(BQ_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

void writeReg16(uint8_t reg, uint8_t lo, uint8_t hi)
{
    Wire.beginTransmission(BQ_ADDR);
    Wire.write(reg);
    Wire.write(lo);
    Wire.write(hi);
    Wire.endTransmission();
}

uint8_t readReg8(uint8_t reg)
{
    Wire.beginTransmission(BQ_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(BQ_ADDR, 1);
    return Wire.read();
}

uint16_t readReg16(uint8_t reg)
{
    Wire.beginTransmission(BQ_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(BQ_ADDR, 2);
    uint8_t lo = Wire.read();
    uint8_t hi = Wire.read();
    return ((uint16_t)hi << 8) | lo;
}

uint8_t getVBUSstat()
{
    return readReg8(REG_CHARGER_STATUS1) & 0x07;
}

// Enables ADC just long enough to take one reading. Called once
// per wake cycle; the caller (sensor_handler) turns the ADC back
// off before returning to sleep.
void initBQ25628()
{
    // Reset watchdog and disable it — default 50s timer halves ICHG on expiry
    uint8_t ctrl0 = readReg8(REG_CHARGER_CTRL0);
    ctrl0 |= 0x04;   // WD_RST = 1
    ctrl0 &= ~0x03;  // WATCHDOG = 00 (disabled)
    writeReg(REG_CHARGER_CTRL0, ctrl0);
    delay(50);

    // Disable external ILIM pin — current controlled by register only
    uint8_t ctrl3 = readReg8(REG_CHARGER_CTRL3);
    writeReg(REG_CHARGER_CTRL3, ctrl3 & ~0x04);

    // Charge current 500mA (ICHG=12=0x0C, field bits 10:5 -> 0x0180)
    writeReg16(REG_ICHG, 0x80, 0x01);

    // Enable ADC: continuous, 12-bit, all channels — only for this reading
    writeReg(REG_ADC_CONTROL, 0x80);
    writeReg(REG_ADC_DISABLE, 0x00);
    delay(200);
}

// ===== All pins to analog for minimum leakage current =====
void setAllPinsAnalog()
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Pin =
        GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 |
        GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 |
        GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 |
        GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_15;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin =
        GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 |
        GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_12;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

// ===== Called by the periodic timer every 10 seconds =====
// Re-inits I2C (pins were analog during sleep), reads sensors,
// then turns the charger's ADC back off before returning.
// Sleeping itself only happens in loop().
void sensor_handler(void *)
{
    pinMode(LED, OUTPUT);
    pinMode(SENSOR_PWR, OUTPUT);

    digitalWrite(LED, HIGH);

    Wire.begin();          // re-init I2C after pins were set to analog
    initBQ25628();         // re-enable charger ADC for this reading

    // --- USB disconnect detection ---
    uint8_t vbus_stat = getVBUSstat();
    if (vbus_stat != 0)
        wasConnected = true;

    if (wasConnected && vbus_stat == 0) {
        Serial.println("USB disconnected — releasing UART2 pins and rebooting...");
        delay(100);
        pinMode(PA2, INPUT); // Release UART2 TX (used by external MUX on other boards)
        pinMode(PA3, INPUT); // Release UART2 RX
        delay(200);
        api.system.reboot();
    }

    // --- SHT40 ---
    if (has_sht40) {
        sensors_event_t humidity, temp;
        sht4x.getEvent(&humidity, &temp);
        Serial.printf("T=%.2f C  RH=%.1f %%\r\n",
            temp.temperature, humidity.relative_humidity);
    }

    // --- BQ25628 ---
    float vbus = ((readReg16(REG_VBUS_ADC) >> 2) & 0x1FFF) * 0.00397f;
    float vbat = ((readReg16(REG_VBAT_ADC) >> 1) & 0x0FFF) * 0.00199f;
    float vsys = ((readReg16(REG_VSYS_ADC) >> 1) & 0x0FFF) * 0.00199f;

    uint16_t ibat_u  = readReg16(REG_IBAT_ADC) >> 2;
    int16_t ibat_raw = (ibat_u & 0x2000) ? (int16_t)(ibat_u | 0xC000) : (int16_t)ibat_u;
    float ibat = ibat_raw * 0.004f;

    uint8_t chg_stat = (readReg8(REG_CHARGER_STATUS1) >> 3) & 0x03;
    const char* chg_str[] = {"Not charging", "Charging CC", "Taper CV", "Top-off"};

    Serial.printf("VBUS=%.3fV VBAT=%.3fV VSYS=%.3fV IBAT=%.0fmA CHG=%s\r\n",
        vbus, vbat, vsys, ibat * 1000, chg_str[chg_stat]);

    // Turn OFF the charger's ADC before sleeping — leaving it in
    // continuous mode keeps it converting on its own and was the
    // single biggest sleep-current leak found in this project
    // (~580uA instead of ~18uA).
    writeReg(REG_ADC_CONTROL, 0x00);

    digitalWrite(LED, LOW);
}

void setup()
{
    pinMode(LED, OUTPUT);
    digitalWrite(LED, LOW);

    pinMode(SENSOR_PWR, OUTPUT);
    digitalWrite(SENSOR_PWR, HIGH); // sensor rail stays powered permanently

    Serial.begin(115200);
    delay(5000); // time for AT+BOOT / firmware flashing

    blink(3, 200);

    Wire.begin();

    has_sht40 = sht4x.begin();
    if (has_sht40) {
        sht4x.setPrecision(SHT4X_HIGH_PRECISION);
        sht4x.setHeater(SHT4X_NO_HEATER);
        Serial.println("SHT40 OK");
    } else {
        Serial.println("SHT40 not found!");
    }

    initBQ25628();
    Serial.println("BQ25628 OK");

    // Create and start the periodic timer — fires sensor_handler every 10s
    api.system.timer.create(RAK_TIMER_0, sensor_handler, RAK_TIMER_PERIODIC);
    api.system.timer.start(RAK_TIMER_0, 10000, NULL);

    sensor_handler(NULL); // take one reading immediately

    // Turn the charger's ADC off again after this first read too
    writeReg(REG_ADC_CONTROL, 0x00);

    api.system.lpm.set(1);
    api.system.lpmlvl.set(2);
}

// loop() is where the actual sleeping happens. The RUI3 scheduler
// re-enters loop() after every wake event (including the periodic
// timer), so this is the only place that should call sleep.all() —
// calling it once at the end of setup() instead means the device
// sleeps exactly once and never sleeps again after the first wake.
void loop()
{
    setAllPinsAnalog();
    api.system.sleep.all();
}
