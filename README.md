# HP Printer LCD Reverse Engineering — AUO 59.02A42.005 + ESP32

Reverse engineering and reuse of the raw RGB LCD from an HP Photosmart TouchSmart control panel.

The original printer mainboard is removed. An ESP32 replaces it as the video source and drives the panel directly.

![Finished clock](docs/img/hero.jpg)

## Project status

Working:

- raw LCD video output from ESP32
- D0-D7 parallel data bus
- DCLK / HSYNC / VSYNC
- full-screen image
- I2S0 LCD mode + DMA
- clock and date
- Wi-Fi + NTP
- Open-Meteo weather
- text, icons and gradients
- RGB/subpixel calibration
- original HP TouchSmart panel housing


---

## Hardware

![Hardware overview](docs/img/overview.jpg)

| Part | Details |
|---|---|
| LCD | AU Optronics `59.02A42.005`, marking `270S06ZS4A06X101620102` / `0244HOO` |
| Related panel family | AUO A024CN02 |
| Resolution | 480 × 234 color dots |
| Interface | 8-bit parallel RGB + DCLK + HSYNC + VSYNC |
| LCD connector | 40-pin FPC |
| HP panel board | `CN245-60001` |
| Panel MCU | Cypress `CY8C20546-24PVXI` |
| Replacement controller | ESP32 DevKit V1 / ESP-WROOM-32 |

---

# Reverse engineering

This project started as a board-level teardown. There was no schematic for the HP panel assembly, so the useful signals were recovered by continuity testing and experimentation.

## 1. Tracing the printer-mainboard connector

The original connector between the TouchSmart panel and the printer mainboard was removed. Its exposed pads were traced to the LCD FPC.

![Host connector pads](docs/img/pads.jpg)

Eleven pads form the raw display interface:

- D0-D7
- DCLK
- HSYNC
- VSYNC

The important discovery was that these signals **do not pass through the Cypress CY8C20546**. They run directly from the printer host connector to the LCD.

That means the original printer mainboard was generating the display stream itself, and the exposed pads can be used as a breakout for another controller.

## LCD pinout recovered from the board

| LCD FPC pin | Signal | ESP32 |
|---:|---|---:|
| 40 | D0 | GPIO13 |
| 39 | D1 | GPIO14 |
| 38 | D2 | GPIO16 |
| 37 | D3 | GPIO17 |
| 36 | D4 | GPIO18 |
| 35 | D5 | GPIO19 |
| 34 | D6 | GPIO21 |
| 33 | D7 | GPIO22 |
| 32 | DCLK | GPIO23 |
| 31 | VSYNC | GPIO26 |
| 30 | HSYNC | GPIO25 |
| 29 | SCL | Cypress MCU |
| 28 | SDA | Cypress MCU |
| 27 | CS | Cypress MCU |

Pins 27-29 remain connected to the Cypress controller in the current build.

## 2. What the CY8C20546 actually does

The Cypress `CY8C20546-24PVXI` initially looked like it might be a display controller.

Continuity testing showed otherwise.

The raw video path bypasses it. The PSoC is connected to:

- LCD `CS`
- LCD `SDA`
- LCD `SCL`
- touch controls
- panel LEDs

The panel displays an image without the ESP32 sending anything over those serial-control lines, so the current firmware leaves them alone.

## 3. Backlight reverse engineering

The LCD initially produced an image but had no backlight. With the printer mainboard removed, `LED_ANODE` remained at 0 V.

![Backlight tracing](docs/img/backlight.jpg)

The area around Q11/Q12 and the six-pin device marked `XI` was traced separately.

For development, the reliable solution is intentionally simple:

**3.3 V -> LED_ANODE**

The backlight is somewhat dimmer than stock, but this avoids guessing at a higher LED voltage while the original driver/control path is still under investigation.

## 4. Finding the correct video format

A valid image did not mean the bus interpretation was correct.

Early timing experiments produced only part of the image:

![Wrong display mode](docs/img/third-of-screen.jpg)

The panel accepted sync, but the incoming RGB stream was being interpreted with the wrong pixel structure.

After timing and subpixel experiments, a complete calibration image could be displayed:

![Calibration](docs/img/calibration.jpg)

The panel is closely related to the AUO A024CN02 family and uses an unusual sequential RGB-dot arrangement rather than a normal packed framebuffer.

RGB phase/order can also differ between alternating rows, so the firmware includes a calibration mode.

## 5. Why GPIO bit-banging was abandoned

Direct GPIO output was useful during the first stage because it proved:

- the pinout
- sync polarity
- active image geometry
- RGB data wiring

But the LCD is very sensitive to interruptions in DCLK. Short CPU pauses caused full-screen white flashes.

Wi-Fi, FreeRTOS scheduling and normal ESP32 interrupt activity therefore made CPU-generated video unreliable.

## 6. I2S LCD mode + DMA

The stable implementation uses the ESP32's **I2S0 peripheral in LCD/parallel mode with DMA**.

The hardware continuously generates:

- D0-D7
- DCLK
- HSYNC
- VSYNC

The CPUs only prepare upcoming line buffers.

This keeps the video stream running while the ESP32 simultaneously handles Wi-Fi, NTP, HTTP weather requests and UI rendering.

---

## Current application

The reverse-engineered display is now used as a small weather clock.

Features include:

- large clock
- date and weekday
- current temperature
- feels-like temperature
- daily min/max
- humidity
- wind direction and speed
- weather description
- weather icons
- day / golden-hour / night palettes
- stars, mountains and sea background
- calibration screen

Weather data comes from Open-Meteo.

---

## Calibration

The calibration screen contains:

- red / green / blue areas
- white reference area
- grayscale gradient
- hue gradient

![RGB calibration screen](docs/img/calibration.jpg)

The firmware supports adjustment of:

- RGB/subpixel phase
- alternating-row mapping
- horizontal mirroring
- vertical mirroring
- row masking
- active-window position

Calibration values can be stored in ESP32 Preferences.

