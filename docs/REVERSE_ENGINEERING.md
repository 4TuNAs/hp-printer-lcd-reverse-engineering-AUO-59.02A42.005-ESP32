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
| 29 | SCL | Cypress MCU (pin later lifted) | not connected |
| 28 | SDA | Cypress MCU (pin later lifted) | not connected |
| 27 | CS | idle at 3.3 V | not connected |

**Result:** `D0–D7`, `DCLK`, `HSYNC` and `VSYNC` bypass the Cypress entirely. The printer mainboard provided an 8-bit video stream. The connector pads are effectively a breakout of the display's raw inputs. The Cypress is associated with the serial-control pins, touch inputs and LEDs; it is **not the video controller**.

The display accepts video without any serial commands from the ESP32: with nothing writing to it, the LCD runs on its default registers, and the default input mode is UPS052 320RGB.

## Cypress disconnected: spontaneous mode switching

With the printer mainboard gone, the CY8C20546 turned out to be a problem. Occasionally after power-on the panel came up in **UPS051** mode instead of UPS052 320RGB:

- the image was stretched about 2.7× horizontally and shifted to the right; only the left quarter of the frame was visible;
- colours were scrambled, because the panel was reading each `0, R, G, B` byte as a separate colour dot;
- the state persisted until the next power cycle. Hot re-plugging the FPC also cleared it, but that risks the LCD driver.

**Diagnosis:** a test mode in the firmware switched the ESP32 output to UPS051 timing (480 one-byte dots per line, 10 MHz, 616-cycle lines). In the bad state this made the picture correct, so the panel itself had changed its input mode. According to the A024CN02 VJ datasheet this is register `R3`, field `SEL`: `000` = UPS051, `001` = UPS052 320RGB (the default).

**Action:** the CY8C20546 is the only other device on the LCD serial-control lines, and it kept switching the panel between modes on its own. Its **`SDA` and `SCL` pins were lifted off the PCB**, so it can no longer clock anything into the LCD. `CS` measures 3.3 V (inactive). The PSoC stays on the board but is cut off from the display; the panel runs on its default registers, which is the UPS052 320RGB mode the firmware expects.

The exact trigger inside the PSoC firmware was not traced further.

**Serial interface for future use** (datasheet section 7): 16-bit frames, MSB first: 4-bit register address, `R/W` bit (`0` = write), 3 don't-care bits, 8 data bits. `CS` is active low, `SCL` idles high, and the panel samples `SDA` on the rising edge of `SCL`. With the Cypress disconnected, the panel-side `SDA`/`SCL` pads can be wired to the ESP32 to write `R3` explicitly at boot.

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
