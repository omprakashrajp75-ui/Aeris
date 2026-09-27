# AERIS

Offline-first health and environmental monitoring on the Arduino UNO Q.
Team THIRAN — Smart India Hackathon 2026, problem statement SIH26181
("A secure, AI-powered Personal Health Companion", Qualcomm Inc.).

Everything runs on the board. No cloud, no internet, no data leaves the device.

## Scope of this build

**This is the idea-submission prototype, not the proposed system.**

It exists to show that the architecture works end to end on real hardware: real
sensors, real derivation on the microcontroller, a real personal baseline, a
real dashboard served by the board, and a real fail-safe that survives the Linux
tier being killed. Every number it displays came from a sensor.

What it deliberately does not yet include is listed under "Planned for the final
build" below. Those components are designed and specified; they are not in this
code. Nothing here is simulated, stubbed, or faked to look more complete than it
is — features that are absent are absent, and values that cannot be read show as
"--".

## What it does today

Reads physiological sensors continuously, derives a strain index against a
personal baseline the device learns on its own, and serves a live dashboard from
a web server running on the board itself. A phone or laptop on the same network
is a viewer, not a dependency.

## Architecture

Two processors, split by responsibility.

**STM32U585 (Cortex-M33)** — `sketch/sketch.ino`
Owns sampling and the alert path. Drains the PPG FIFO, derives heart rate and
SpO2, classifies motion state, reads temperature. Drives the on-board RGB LED
and 8x13 LED matrix directly. Keeps alerting when Linux is gone.

**QRB2210 (Cortex-A53, Debian)** — `python/`
Owns the slow, stateful work: personal baseline, strain indices, SQLite
persistence, and the dashboard. Polls the M33 once per second over Bridge RPC.

The split is the point. Linux can crash, be killed, or be updated, and the
wearer still gets alerted. This is the property the final build scales up, not
a demo shortcut.

## Hardware in this build

| Part | Bus | Address / pin | Provides |
|---|---|---|---|
| MAX30102 | I2C | 0x57 | Red + IR PPG → heart rate, SpO2 |
| TMP102 | I2C | 0x48 | Skin temperature |
| MPU6050 | I2C | 0x68 | 3-axis accelerometer → motion state |
| DHT11 | GPIO | D2 | Ambient temperature + humidity |

All modules run at 3.3 V on the header SDA/SCL pins (`Wire`).
On-board RGB LED4 (active low) and the 8x13 LED matrix are driven by the M33.

Bring-up notes worth keeping:
- Many MPU6050 modules are clones and report WHO_AM_I values other than 0x68
  (0x70 is common). The init accepts the known clone IDs.
- The MAX30102 runs at reduced LED current (0x1F) with 4x hardware sample
  averaging at 100 Hz, giving ~25 Hz of averaged output.
- The DHT11 data line needs a pull-up to 3V3 if the module does not carry one.

## Planned for the final build

Specified and designed, not present in this code:

- **Full sensor stack** — MAX30101 (green-channel PPG for wrist-worn use),
  TMP117 (±0.1 °C skin temperature), SHT4x (ambient temperature and humidity),
  BMV080 (particulate matter), BMP390 (barometric pressure), SCD41 (CO2).
- **Per-channel statistical anomaly detection on the M33** — EWMA mean and
  variance per channel, motion-state-conditioned for physiological channels,
  residual-scored for slow-drifting environmental channels, composited into a
  severity level. This build grades heart rate against cached thresholds only.
- **Correlative health-environment parameters** — heat stress, dehydration risk,
  fatigue, respiratory load, environmental exposure and cardiovascular strain,
  each fused from several sensors rather than read from one.
- **On-device LLM narrative tier** — a quantised sub-billion-parameter model via
  llama.cpp on the QRB2210, generating plain-language summaries from stored
  data. Strictly off the alert path: it explains decisions, it never makes them.
- **Companion application over BLE** — trends, history, GPS-tagged SOS and a
  clinician-facing export.
- **Calibration** — ECTemp coefficients fitted against the published reference,
  and SpO2 verified against a reference oximeter.

## Bridge RPC

| Method | Direction | Purpose |
|---|---|---|
| `read_vitals` | Python → M33 | Current sensor values and validity flags, as JSON |
| `set_thresholds` | Python → M33 | Cache baseline-derived alert bands in the M33 |
| `raise_alert` | Python → M33 | Trigger the alert output path |
| `drain_autonomous_log` | Python → M33 | Collect alerts buffered while Linux was down |
| `ack_alert` | M33 → Python | Acknowledgement notification |

## Dashboard

FastAPI WebUI Brick on port 7000, reachable at the board's LAN address.
Four tabs: Home (live vitals and strain), Trends (8 h chart), Baseline
(resting-HR learning progress), Alerts (today's alerts, each markable as a
false alarm).

Live values push over WebSocket once per second. socket.io is vendored locally
at `assets/libs/socket.io.min.js` on purpose — the dashboard must render with no
internet connection. Do not replace it with a CDN link.

Any value whose validity flag is false renders as "--". The dashboard never
shows a stale number as if it were live.

## Running it

Open the app in Arduino App Lab and press Start. App Lab compiles and flashes
the M33 sketch and starts the Python tier. The dashboard address is printed at
startup.

## Fail-safe behaviour

If no Bridge call arrives for 30 seconds the M33 enters autonomous mode: it
keeps sampling, grades heart rate against the last cached thresholds, keeps
driving the RGB LED and LED matrix, and buffers alerts in a ring buffer. On
reconnection Python drains that buffer via `drain_autonomous_log`.

To demonstrate:

    pkill -f aeris

The Python tier dies. The board keeps alerting.

## Honest limits of this prototype

Stated plainly because a reviewer will find them anyway.

- **Core temperature is an estimate, not a measurement.** The ECTemp Kalman
  filter coefficients in `python/aeris/indices.py` are placeholders, not fitted
  values. `CALIBRATED = False`, and full PSI is deliberately excluded from alert
  grading — alerts run on the heart-rate term (`psi_hr_only`) alone.
- **SpO2 is uncalibrated.** Ratio-of-ratios with no reference oximeter to verify
  against. Treat it as a trend, not a clinical number.
- **Sensors are demo substitutes.** TMP102 (±0.5 °C) stands in for TMP117, and
  DHT11 for the SHT4x. The TMP102's error is larger than some of the
  skin-temperature deviations being detected.
- **Wrist PPG is weak.** The MAX30102 has no green LED — red and IR only, which
  perform poorly through wrist tissue under motion. Demo readings are fingertip.
- **Anomaly detection is threshold-based, not statistical.** See "Planned for
  the final build".
- **No accuracy validation has been performed.** Nothing here has been measured
  against ground truth. This build demonstrates that the pipeline works, not
  that its numbers are right.
- **Alerts are local.** Device-side notification only. No emergency service is
  contacted.

## Layout

    sketch/sketch.ino     M33 firmware: sensors, derivation, alert path
    python/main.py        Bridge poll loop, SQLite, alert evaluation
    python/aeris/         ECTemp, PSI, PersonalBaseline
    python/web/server.py  Dashboard REST and WebSocket handlers
    assets/               Dashboard HTML, JS, CSS
    data/aeris.db         SQLite store (readings, indices, alerts)
