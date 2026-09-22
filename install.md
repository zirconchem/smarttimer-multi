# Installation Guide

## Part 1 — ESP32 firmware

### Hardware
- ESP32 dev board (any variant)
- DS3231 RTC module (I2C, address 0x68)
- Relay module (active-low or active-high)
- Optional: LED on GPIO 2 for status

### Wiring
| ESP32 | DS3231 | Relay |
|-------|--------|-------|
| GPIO21 | SDA | — |
| GPIO22 | SCL | — |
| GPIO27 | — | IN |
| GPIO2 | — | status LED |
| 3.3V | VCC | VCC |
| GND | GND | GND |

### Flashing
1. Open `SmartTimer.ino` in Arduino IDE 2.3.8.
2. Install libraries via Library Manager:
   - RTClib (Adafruit)
   - ArduinoJson (Benoit Blanchon)
   - PubSubClient (Nick O'Leary)
3. Select board: **ESP32 Dev Module**.
4. Upload.

### First boot
1. Phone → Wi-Fi → connect to `STimer2` (password `timer456`).
2. Browser → `http://smarttimer.home` or `http://192.168.4.1`.
3. Go to **Wi-Fi Setup** → enter your home router's SSID/password → Test & Save.
4. Now the ESP32 is on your home network too. Access it via `http://smarttimer.local` or its router IP.

## Part 2 — EMQX Cloud broker (free tier)

1. Sign up at https://cloud-intl.emqx.com (no credit card).
2. Create a **Serverless deployment**.
3. Set **Spend Limit = 0** in billing.
4. Under **Authentication**, create a username/password for your device.
5. Note from the Overview page:
   - **Address** (e.g., `hc1c11bb.ala.asia-southeast1.emqxsl.com`)
   - **MQTT over TLS Port**: `8883`
   - **WebSocket over TLS Port**: `8084`

## Part 3 — Configure ESP32 MQTT

1. On the device's web UI → **MQTT Setup**.
2. Fill:
   - Broker host (from above)
   - Port: `8883`
   - Username / Password (from EMQX)
   - Client ID: leave default `smarttimer-esp32`
   - Topic base: leave default `smarttimer`
3. Click **Test & Save** — should say "Connected".

## Part 4 — Android app

### Option A: Use the pre-built APK (if you already have it)
1. Install `app-debug.apk` on your phone.
2. Open → enter broker details → Save & Connect.

### Option B: Build your own APK via GitHub
1. Create a public repo on GitHub.
2. Upload the `Android-App/` folder contents (see folder structure above).
3. Commit → Actions tab → wait ~8 minutes → green check.
4. Click the run → scroll to Artifacts → download `smarttimer-debug-apk.zip`.
5. Unzip → install `app-debug.apk` on your phone.

## Part 5 — Daily use

**At home**: open `http://smarttimer.local` (or IP) in browser for full config.

**Away**: open the Android app for quick ON/OFF/CLEAR override and status.

## MQTT topics (reference)

### Device publishes
| Topic | Values |
|-------|--------|
| `smarttimer/status/relay` | `ON` / `OFF` |
| `smarttimer/status/mode` | `Schedule` / `Override` |
| `smarttimer/status/override` | `none` / `on_indefinite` / `on_until_HH:MM` |
| `smarttimer/status/time` | `YYYY-MM-DD HH:MM:SS` |
| `smarttimer/status/source` | `DS3231` / `NTP` / `System` |
| `smarttimer/status/temp` | Celsius |
| `smarttimer/status/last_ntp` | timestamp or `never` |
| `smarttimer/status/online` | `online` (LWT = `offline`) |

### Device subscribes
| Topic | Payloads |
|-------|----------|
| `smarttimer/cmd/override` | `on`, `off`, `clear`, `on:60`, `off:30` |

## Troubleshooting

**AP but no internet / router not connecting**
- Wrong Wi-Fi password → reconnect to `STimer2` AP, re-enter credentials.

**`smarttimer.local` not resolving**
- Some Android browsers are picky about mDNS. Use the router IP instead (see Network Info page).

**MQTT shows "connect fail"**
- Check host, port (8883), username, password.
- Ensure EMQX Authentication is enabled and the user exists.
- Ensure Spend Limit is not blocking.

**APK build fails on GitHub**
- Make sure the workflow uses `android-actions/setup-android@v4` with `packages: ''`.

**Device offline in app but powered on**
- Check the ESP32's Network Info page → MQTT section → status should be `connected`.

## Safety notes

- The AP `STimer2` is always on with default password. Change it in the sketch if security matters.
- MQTT credentials live in the app's local storage — anyone with physical access to the phone can read them.
- Use TLS (port 8883 / 8084) only. Never plain 1883 over the internet.
- EMQX Spend Limit = 0 prevents surprise charges.