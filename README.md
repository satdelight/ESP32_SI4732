# SI473x on ESP32 — Full-Feature Receiver with Real RefClk and PPM Diagnostics

An ESP32-based receiver frontend for Silicon Labs SI473x (e.g., SI4732/35) inspired by and based on examples from PU2CLR (Ricardo Lima Caratti). In addition to using the actual measured reference clock with PPM diagnostics, the sketch provides a complete multi-band, multi-mode UI with OLED (and optional TFT), RDS handling, EEPROM persistence, and extensive tuning controls.

- PU2CLR SI4735 Arduino Library: https://github.com/pu2clr/SI4735
- PU2CLR Documentation: https://pu2clr.github.io/SI4735/

If you find this useful, please support and star the original PU2CLR work.

## Key features

Radio and modes
- Modes: FM, AM, LSB, USB
- SSB support with BFO and selectable SSB audio bandwidth; automatic sideband cutoff selection depending on chosen SSB bandwidth
- AM/FM bandwidth control (per-mode bandwidth tables)
- Adjustable tuning step sizes and per-band seek spacing

Bands (curated presets)
- Broadcast: LW, MW (EU 9 kHz and NA 10 kHz spacings), and a comprehensive set of SW broadcast bands (incl. 120m, 90m, 75m, 49m, 41m, 31m, 25m, 22m, 19m, 16m, 15m, 13m, 11m)
- Amateur: 630m, 160m, 80m, 60m, 40m, 30m, 20m, 17m, 15m, 12m, 10m
- CB: Standard 40-channel and DE 80-channel variants
- Catch-all: 150 kHz–30 MHz general band
- Region-aware MW spacing (9/10 kHz) and grid alignment helpers for CB/others

UI and displays
- Rotary encoder with push button for tuning and menu navigation
- OLED 128×32 (Adafruit SSD1306) frequency screen with band/mode and live RSSI/SNR
- Optional TFT ST7735 full display (status, frequency, bandwidth, RSSI, AGC/ATT, etc.)

RDS (FM)
- Program Service (PS) name with debounce
- RadioText (RT) with debounce and scrolling window
- On/off toggle from the UI; resets PS/RT buffers on frequency change

Signal control
- AGC on/off with manual attenuation levels
- Soft-mute maximum attenuation tweak (useful to manage FM noise)
- Volume control

Antenna and region
- Antenna capacitor control (ANTCAP): Auto vs. Hold; for AM/SW/CB retune is applied to immediately take effect
- AM region selection: EU 9 kHz vs. NA 10 kHz; seek spacing derived per band

Persistence
- EEPROM-backed storage of settings: current band/frequency, per-band step, bandwidth index, volume, AGC/attenuation, AM region, ANTCAP mode, and more
- Automatic load at boot; periodic save while running

Reference clock and diagnostics
- Uses an ESP32 LEDC channel to output and measure the real reference clock
- The measured ref clock (g_realRefHz) is applied to the SI473x with an optional static trim (REFCLK_TRIM_HZ)
- Serial diagnostics at startup: measured vs. nominal 32,768 Hz, PPM mismatch, and equivalent error at a sample RF frequency

## Hardware

- ESP32 development board
- SI473x module/breakout (e.g., SI4732/35)
- I2C bus with pull-ups (commonly on the module)
- Antenna and audio per your SI473x board
- Optional:
  - 128×32 OLED (SSD1306)
  - ST7735 TFT

Default pins (adjust for your board)
- I2C: SDA = 21, SCL = 22
- SI473x RESET: GPIO 12
- Rotary encoder: A = 13, B = 14, Push = 27
- Ref clock out: GPIO 25 (LEDC-based square wave)

## Software setup

Arduino IDE
1. Install “ESP32 by Espressif Systems” via Boards Manager.
2. Install PU2CLR SI4735 library via Library Manager.
3. Select your ESP32 board and port.
4. Open ESP32_SI4732.ino, compile, and upload.
5. Open Serial Monitor at 115200 baud.

PlatformIO
1. Open the project folder in VS Code with PlatformIO.
2. Ensure platformio.ini targets your ESP32 board.
3. Build and upload: `pio run -t upload`
4. Serial monitor: `pio device monitor -b 115200`

## Configuration highlights (in code)

- Reference clock
  - REFCLK_HZ = 32768.0
  - REFCLK_TRIM_HZ = 0 (set ±Hz to compensate)
  - LEDC channel/bit depth for clock output
- Pins: RESET_PIN, ENCODER pins, OLED/TFT usage flags
- Band tables, steps, and bandwidth presets
- EEPROM size/layout and application ID

## Operation

- Tuning
  - Rotate encoder to change frequency (VFO); FM resets RDS buffers on change
  - Step size and bandwidth selectable per mode
  - Seek spacing automatically set per band

- Menu and actions
  - Single click: context action (e.g., toggle/tune depending on current command)
  - Double click: enter menu; rotate to choose, click to confirm
  - Common items: Band select, Volume, AGC/ATT, Bandwidth, Step, Mode, Soft-mute Max Att, RDS On/Off, AM Region (9/10 kHz), ANTCAP Auto/Hold, BFO (SSB)

- SSB and BFO
  - LSB/USB modes with BFO adjustment (fine steps)
  - SSB audio bandwidth selection; automatic sideband cutoff filter depending on chosen bandwidth

- RDS
  - Toggle RDS in FM; PS and RT are debounced to reduce flicker; RT is scrolled when long

- RSSI/SNR
  - Periodically polled and shown on OLED (and TFT if enabled)

## Reference clock PPM diagnostics

On boot, the serial log shows, for example:
```
Measured ESP32 LEDC ref:      32767.85 Hz
Nominal:                      32768.00 Hz
Delta:                         -0.15 Hz  =>  -4.6 ppm
RefClk sent to SI473x:        32767.85 Hz (+0 Hz trim)
Equiv. error @ 3.955 MHz:     -18.1 Hz
```
Use REFCLK_TRIM_HZ to minimize your observed error for your target bands. For precision work, verify against known signals or a frequency standard.

## Troubleshooting

- No I2C device
  - Check wiring/voltage and that pull-ups are present
- Constant frequency offset
  - Review PPM diagnostics and apply REFCLK_TRIM_HZ
- Excessive FM noise
  - Adjust soft-mute max attenuation and verify antenna/grounding
- SSB audio/distortion
  - Try different SSB bandwidths and verify sideband cutoff
- Settings not restored
  - Ensure EEPROM size is sufficient and application ID matches the sketch

## Credits

- Based on examples by PU2CLR — Ricardo Lima Caratti
  - https://github.com/pu2clr/SI4735
  - https://pu2clr.github.io/SI4735/

## License

See the LICENSE file in this repository (or add one, e.g., MIT, if not present).