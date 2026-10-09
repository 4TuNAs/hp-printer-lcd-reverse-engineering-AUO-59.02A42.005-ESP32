# Reverse-engineering the HP Photosmart TouchSmart LCD board

This document is the hardware notebook behind the [main project](../README.md). It records which connections were **confirmed by continuity testing** and which component identifications remain **tentative**.

![Panel board, LCD and wired ESP32](img/overview.jpg)

## Original assembly

- HP Photosmart TouchSmart panel PCB marked **`CN245-60001`**.
- AUO LCD module marked **`59.02A42.005`**, related to the **A024CN02 VJ** family.
- 40-pin LCD FPC connector.
- Cypress **`CY8C20546-24PVXI`** PSoC near the touch controls / panel LEDs.
- Printer-mainboard connector (removed for tracing).

The longer label string `270S06ZS4A06X101620102` is omitted because the photograph does not establish it reliably and it is not required for reusing the interface.

## Finding the video interface

The printer-mainboard connector was desoldered to expose copper pads. Each pad was checked for continuity to the 40-pin display FPC.

![Former printer-mainboard connector, signals and ESP32 wires](img/pads.jpg)

| FPC pin | Recovered function | Destination | ESP32 GPIO |
|---:|---|---|---:|
| 40 | D0 (LSB) | Printer connector | 13 |
| 39 | D1 | Printer connector | 14 |
| 38 | D2 | Printer connector | 16 |
| 37 | D3 | Printer connector | 17 |
| 36 | D4 | Printer connector | 18 |
| 35 | D5 | Printer connector | 19 |
| 34 | D6 | Printer connector | 21 |
| 33 | D7 (MSB) | Printer connector | 22 |
| 32 | DCLK | Printer connector | 23 |
| 31 | VSYNC | Printer connector | 26 |
| 30 | HSYNC | Printer connector | 25 |
| 29 | SCL | Cypress MCU | not connected |
| 28 | SDA | Cypress MCU | not connected |
| 27 | CS | Cypress MCU | not connected |

**Result:** `D0–D7`, `DCLK`, `HSYNC` and `VSYNC` bypass the Cypress entirely. The printer mainboard provided an 8-bit video stream. The connector pads are effectively a breakout of the display's raw inputs. The Cypress is associated with the serial-control pins, touch inputs and LEDs; it is **not the video controller**.

The display was observed to accept video without sending additional serial commands from the ESP32. This does not prove that the LCD never needs initialization after a complete unpowered reset; the Cypress remains on the board.

**Bus contention:** do not connect an ESP32 output directly to `CS/SDA/SCL` while the CY8C20546 is also driving them. Its exact protocol has not been decoded here.

## Power: 3.3 V rail and FPC pins 1–26

A 3.3 V supply applied to the **wide 3.3V trace** on the original HP panel board powers enough of the on-board circuit for the LCD to operate. The existing analog/charge-pump components around FPC pins 1–26 are kept in place; they do not need individual external wires.

Those pins include functions such as **VCC, AVDD, PVDD, VCOM, VGH/VGL, charge-pump capacitor connections and LED-related terminals** on this panel family. This is a functional grouping, **not** an asserted pin-by-pin mapping for pins 1–26. Consult the A024CN02 VJ datasheet and measure the exact revision before bypassing any of the original circuits.

During the test, a **bulk electrolytic capacitor** and a **0.1 µF ceramic capacitor** were added near the board supply input. Polarity matters for the electrolytic; power the original rail at **3.3 V**, not directly at USB 5 V.

## Backlight path

The LCD drew a picture without backlight illumination. With the printer mainboard absent, `LED_ANODE` read approximately 0 V.

![Backlight board area, LED anode and transistor traces](img/backlight.jpg)

Hardware tracing indicated the following arrangement:

- Two SOT-23 devices **Q11/Q12**, marked **`A2` / `2A`**, appear wired in parallel: their base/control connections route towards the printer connector; one common net goes to `LED_ANODE` and another to the six-pin device marked `XI`.
- The **`XI`** part appears consistent with a **dual NPN digital transistor**, possibly Toshiba **RN1608** or a similar device. This is a working hypothesis, **not a verified part identification**.
- Diode-test readings noted **≈ 660 mV** (pin 2 → 1) and **≈ 650 mV** (pin 5 → 4), open in the opposite test direction. A possible layout is `1=E1`, `2=IN1`, `6=C1`, `4=E2`, `5=IN2`, `3=C2`; confirm against the actual component before designing around it.
- **L3** was traced and did not form part of the backlight supply route; an earlier boost-converter hypothesis was rejected.

The related AUO datasheet lists a typical backlight rating around **3.8 V / 25 mA**. In this experiment, a temporary jumper from **3.3 V to `LED_ANODE`** illuminated it more dimly than the original. This is a recorded lab connection, **not** a generic recommendation to apply voltage directly to other LED modules. A permanent design should use controlled LED current after the specific panel has been characterized.

## Reconstructing timing

The initial bit-bang test proved connectivity but showed only part of the intended width and occasional white-screen flashes. The crucial recognition was the **UPS052 320RGB** input protocol, not a simple 480-byte-wide UPS051 stream.

![Partial frame under the wrong interpretation](img/third-of-screen.jpg)

In UPS052, **320 panel pixels × 4 DCLK samples** (`0, R, G, B`) occupy **1280 cycles** of each active line. The current firmware starts the active region at sample 256 within a **1544-cycle line**, with a **16 MHz** DCLK, and the panel handles its own delta colour-dot geometry.

![Working full-width calibration frame](img/calibration.jpg)

See the [line timing diagram](img/line-timing.svg) and the [hardware video pipeline diagram](img/block-diagram.svg); both correspond to the published UPS052 code, not the earlier 10 MHz experimental firmware.

## Scope of the measurements

Continuity and operation establish the digital bus and 3.3 V boot method. Q11/Q12 transistor function and the exact `XI` manufacturer/device remain provisional; the exact individual FPC power-pin assignment was **not** fully traced. Where a part identity is inferred, this document explicitly marks it as such.
