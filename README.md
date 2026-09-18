# HiveNode RUI3 Examples

Arduino/RUI3 example sketches for the HiveNode board — a RAK3172-T
(STM32WLE5) based LoRaWAN node with GPS, Bluetooth LE, RS485, and a
set of onboard I2C sensors, built for long battery-life outdoor
deployments (beehive monitoring, asset tracking, environmental
sensing).

More info: https://hivenode.net

## Contents

| Folder | What it demonstrates |
|---|---|
| `example_01_sensors_sleep/` | SHT40 + BQ25628 read on a timer, with correct deep-sleep behaviour (~18µA) |
| `example_02_gps/` | ATGM336H-5NR-32 GPS via the shared UART multiplexer |
| `example_03_bluetooth/` | E104-BT52 BLE module, AT command interface, via the same multiplexer |
| `example_04_rs485_modbus/` | RS485 Modbus RTU master, via the same multiplexer |

Each example is self-contained and can be opened directly in the
Arduino IDE with the RAKwireless RUI3 board package installed
(Board Manager URL:
`https://raw.githubusercontent.com/RAKwireless/RAKwireless-Arduino-BSP-Index/main/package_rakwireless_index.json`,
board: **WisDuo RAK3172-T Board**).

## Hardware overview

### Core module

- **RAK3172-T** (STM32WLE5), programmed through RUI3 (Arduino core)
- USB-C for power/programming, RS485 or battery for standalone use

### Power path — BQ25628 (TI Li-ion charger, I2C address `0x6A`)

- `VBUS` — USB input
- `PMID` — intermediate rail feeding the board's LDOs
- `VBAT` — single-cell Li-ion battery
- I2C is shared with the sensor bus (`PA11`=SDA, `PA12`=SCL on
  RAK3172), pulled up by the sensor breakout boards themselves — no
  extra pull-up resistors are on this board.
- **Part ID read from the chip is `0x30`**, not `0x22` as assumed by
  the Adafruit BQ25628E Arduino library (which is written for a
  different chip revision and refuses to `begin()` on this part).
  All BQ25628 access in these examples is done with raw
  `Wire.beginTransmission()`/register reads — see
  `example_01_sensors_sleep.ino` for the full register map used.
- The charger's own ADC (`REG_ADC_CONTROL`, `0x26`) must be turned
  off between readings if you care about sleep current — left in
  continuous mode it adds ~560µA on its own, dwarfing everything
  else on the board.

### Sensors

- **SHT40** — temperature/humidity, I2C address `0x44` (Adafruit
  SHT4x library)
- **LIS2DW12** / **LIS3DH** — accelerometer, I2C address `0x18`
  (see project notes; LIS3DH via SparkFun's library is the more
  reliable option of the two on this board)
- **BQ25628** — also exposes battery/charger telemetry over the
  same I2C bus (see above)

### GNSS — ATGM336H-5NR-32

- UART, 9600 baud, NMEA 0183
- Patch antenna (GPS1003) through an AT2659S LNA
- The `$GPTXT,...,ANTENNA OPEN` message is a benign warning caused
  by the passive-antenna-through-LNA setup not matching the
  current-sense check the module expects for an "active" antenna —
  fixes work normally despite it.

### Bluetooth — Ebyte E104-BT52

- BLE 5.0, DA14531-based UART-to-BLE bridge
- **AT command UART runs at 115200 baud**, not the more commonly
  assumed 9600
- `WKP` pin needs a short HIGH pulse (not a held level) to force
  AT-command mode reliably
- Responds `+OK` / `+OK=<value>` to real commands — a bare `OK` or
  `AT_COMMAND_NOT_FOUND` on the console means RUI3's own AT parser
  intercepted the command, not the BT module (fixed by opening
  `Serial1` with `RAK_CUSTOM_MODE`)

### RS485

- Half-duplex, transceiver `DE`+`RE#` tied to a single `DIR` GPIO
- 9600 baud, Modbus RTU (function 0x03 in these examples)

### Shared UART multiplexer — TS3A5017DR

GPS, Bluetooth, and RS485 all share RAK3172's `Serial1` (UART1,
`PB6`=TX, `PB7`=RX) through a TS3A5017DR analog multiplexer, so
only one of the three can be active at a time.

Control pins:

| Signal | RAK3172 pin |
|---|---|
| `MUX_EN` (active **LOW**) | `PB2` |
| `MUX_IN1` | `PA15` |
| `MUX_IN2` | `PA10` |

Confirmed channel map (verified with a multimeter — do not trust
the datasheet's `S1`..`S4` numbering by inspection alone, it does
not line up with a naive 0-indexed reading of `IN2`/`IN1`):

| `IN2` | `IN1` | Device |
|---|---|---|
| L | L | Bluetooth (E104-BT52) |
| L | H | RS485 |
| H | L | GPS (ATGM336H) |
| H | H | *(unused)* |

Because GPS/RS485 need 9600 baud and Bluetooth needs 115200,
`Serial1` must be closed (`Serial1.end()`) and reopened
(`Serial1.begin(<baud>, RAK_CUSTOM_MODE)`) whenever you switch which
device is selected on the multiplexer. `RAK_CUSTOM_MODE` is
important on its own even if you never switch devices — without it,
RUI3's built-in AT command parser watches the UART and swallows
anything that looks like an AT command before it reaches the
external device.

### USB / power-indicator LED note

On earlier board revisions with an external UART multiplexer wired
this way, disconnecting USB while running on battery could leave a
power-indicator LED lit, because the multiplexer's control lines
(`PA2`/`PA3`, UART2) stayed driven and kept a downstream rail
energized. If you see this on your revision, release those pins to
`INPUT` before rebooting on USB disconnect — see the
`checkUSB()`/reboot logic in `example_01_sensors_sleep.ino` for the
pattern.

## Example 01 — Sensors + Sleep

The most important example in this set, and the one that took the
most iteration to get right. Full details are in the comment block
at the top of `example_01_sensors_sleep.ino`, summarized here:

- **`loop()` must be the only place that calls
  `api.system.sleep.all()`** (together with `setAllPinsAnalog()`).
  The RUI3 scheduler re-enters `loop()` after every wake event —
  including periodic timer ticks — so if the sleep call lives only
  at the end of `setup()`, the device sleeps exactly once and stays
  awake forever after the very first wake.
- The timer callback (`sensor_handler`) must call `Wire.begin()`
  again on every invocation, because `setAllPinsAnalog()` puts the
  I2C pins into analog mode during sleep.
- **BQ25628's ADC must be turned off** (`REG_ADC_CONTROL = 0x00`)
  at the end of every wake cycle. Left in continuous mode, it alone
  raised measured sleep current from ~18µA to ~580µA — by far the
  largest single current leak found while building this board's
  firmware.
- Measured sleep floor: **~18µA**. Waking every 10 seconds to read
  both sensors pulls the *average* current up substantially because
  each active phase briefly draws several mA — for real deployments,
  increase the timer interval (60–600s) to bring the duty-cycle
  average down; it does not affect the sleep floor itself.

## License

MIT License — see the header of each `.ino` file. Free to use,
modify, and redistribute; provided as-is with no warranty.
