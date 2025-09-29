# BME280 over I²C on ESP-IDF (new `i2c_master` API)

Tiny, focused example that drives a **Bosch BME280** (temperature / pressure / humidity) via the **new ESP-IDF I²C master driver**.  
It shows four practical acquisition patterns (forced one-shot, forced burst with median, normal/pulsed, normal/continuous) and the plumbing the Bosch C driver expects (read/write/delay callbacks).

---

## Highlights

- Uses the new **`i2c_master_*`** APIs (ESP-IDF 5.x+).
- Four measurement modes you can toggle at compile time.
- Burst mode computes a **median** per field to reduce outliers.
- Works at **400 kHz Fast-mode** I²C.

---

## Measurement modes

| Mode                      | Power mode on sensor | When conversion runs                                  | Typical use                                   |
|---------------------------|----------------------|-------------------------------------------------------|-----------------------------------------------|
| `FORCED_PERIODIC_ONE_TIME`| FORCED (one-shot)    | Triggered once per period                             | Low-rate logging, lowest average power        |
| `FORCED_PERIODIC_BURST`   | FORCED (repeated N)  | `BURST_COUNT` quick one-shots; **median** is logged   | Suppress turbulence/spikes, moderate power    |
| `NORMAL_PERIODIC`         | NORMAL (short pulse) | Wake to NORMAL, wait one cycle, read, back to SLEEP   | Periodic slotting with decent latency         |
| `NORMAL_CONTINUOUSLY`     | NORMAL (steady)      | Sensor free-runs; app polls at 1 s                    | Live dashboards; simplest latency profile     |

Each mode has its own sensor settings (oversampling / IIR / standby) chosen as sensible defaults:

- **Forced one-shot:** `osr_t=2x`, `osr_p=8x`, `osr_h=2x`, **IIR off**
- **Forced burst:** `osr_t=2x`, `osr_p=8x`, `osr_h=2x`, **IIR=2**
- **Normal periodic:** `osr_t=2x`, `osr_p=4x`, `osr_h=2x`, **IIR=4**, `standby=125 ms`
- **Normal continuous:** `osr_t=2x`, `osr_p=16x`, `osr_h=2x`, **IIR=4**, `standby=500 ms`

---

## Configuration
All user-tunable parameters live in /main/config.h for easy tweaking.
You can change:
- I²C settings: port, SDA/SCL pins, bus frequency (400 kHz by default)
- Stack buffer size used for short writes
- Measurement mode: choose between FORCED_PERIODIC_ONE_TIME, FORCED_PERIODIC_BURST, NORMAL_PERIODIC, NORMAL_CONTINUOUSLY
- Timing parameters: report period, burst sample count, median enable, extra margin for NORMAL pulse mode

---

## Hardware

- **ESP32** dev board  
- **BME280** breakout (I²C, **0x76** by default)  
- Wires:  
  - `SDA` → **GPIO 23** (default in code)  
  - `SCL` → **GPIO 22** (default in code)  
  - `VCC` → 3V3  
  - `GND` → GND  
- Pull-ups: example enables **internal pull-ups**. External 4.7–10 kΩ are still recommended for long wires or noisy environments.

> Adjust pins/addr in the code if your wiring differs.

---
