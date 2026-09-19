# Arduino IDE build

The same D1 firmware as `firmware/`, packaged as an Arduino sketch. Pick one —
you do not need both.

| | ESP-IDF (`firmware/`) | Arduino (this folder) |
|---|---|---|
| entry point | `app_main()` in `main.c` | `setup()` in `framing_rover.ino` |
| network | lwIP sockets, `net.c` | `WiFiUDP`, `net_arduino.cpp` |
| hardware | ESP-IDF drivers, `hal_esp32.c` | Arduino core, `hal_arduino.cpp` |
| everything else | shared, from `firmware/main/` | the same files, copied in |

`firmware/main/` is the single source of truth. The `.c`/`.h` files in
`framing_rover/` are **copies** carrying a "do not edit" banner —
`tests/test_arduino_sketch.py` fails the moment they drift.

```
python tools/gen_arduino_sketch.py           # refresh the copies
python tools/gen_arduino_sketch.py --check   # verify
make -C firmware_arduino/test                # compile it without the IDE
```

---

## What to open

**`framing_rover/framing_rover.ino`.** That one file. Arduino IDE opens every
other file in the folder automatically as tabs — you do not open them
individually, and the folder name has to stay the same as the `.ino`.

---

## Before you flash

### 1. Board support

Boards Manager → install **esp32 by Espressif Systems**. Core 2.x and 3.x both
work; the sketch handles the LEDC and ADC API differences between them.

No external libraries are needed for the firmware as it stands. Adding the
range sensor at bring-up step 9 needs one — see *Range sensor* below.

### 2. Wi-Fi credentials

```
cd firmware_arduino/framing_rover
cp secrets_example.h secrets.h      # then edit it
```

`secrets.h` is gitignored. The rover joins as an ordinary station and takes a
DHCP address; the laptop has to be on the same network.

### 3. The pin map — do this one properly

Open the `app_config.h` tab and check every pin against **your** board's
schematic. "ESP32-S3-CAM" covers several layouts and the values in there are a
starting point, not an answer.

The `ROVER_CHECK_PIN` assertions catch reserved GPIOs at compile time
(strapping, USB, UART0, flash, and PSRAM if you uncomment
`ROVER_OCTAL_PSRAM`). They cannot know about your board's camera socket or
on-board LED, so they are a backstop, not a substitute for reading the sheet.

The firmware needs 10 GPIO: 2 PWM, 4 direction, 4 stepper. If your board cannot
spare 10, move the stepper's four lines to a PCF8574 I²C expander — a 28BYJ-48
tops out near 500 half-steps/s, well inside a 100 kHz bus.

### 4. Tools menu

| Setting | Value | Why |
|---|---|---|
| Board | **ESP32S3 Dev Module** | |
| USB CDC On Boot | **Enabled** | otherwise `Serial` prints nothing and you never see the rover's IP |
| CPU Frequency | **240 MHz** | the 1 kHz tilt tick and 100 Hz drive task want the headroom |
| Flash Size | as fitted | usually 8 MB or 16 MB |
| PSRAM | as fitted | **OPI PSRAM** on most S3-CAM boards |
| Partition Scheme | Default 4 MB with spiffs | anything with ≥1.2 MB app |
| Core Debug Level | **Info** | or the `ESP_LOGI` lines never print |
| Upload Speed | 921600 | drop to 115200 if uploads fail |
| Arduino Runs On / Events On | Core 1 / Core 0 | the default, and what F-1 assumes |

### 5. Electrical, before power

**Never power the ESP32 from the L298N's onboard 5 V regulator.** Separate buck
converter, common ground, bulk capacitance at the ESP32. An L298N drops roughly
1.8–2 V, so a 7.4 V pack gives the motors about 5.5 V and a deadband of
25–40% duty — which is why `min_duty_left/right` is a mandatory calibration,
not an optional one.

---

## Flash it

Upload, then **open Serial Monitor at 115200**. You are looking for:

```
=======================================================
  rover IP: 192.168.1.47
  listening on udp/3333, protocol v1, config a0114ebe
  put this IP in config.json -> net.firmware_host
=======================================================
```

**Write that IP down.** The firmware never initiates — it only replies to
whoever sends it a valid command — so the laptop needs it:

```
python -m app.run --host 192.168.1.47
```

or set `net.firmware_host` in `config.json`. The committed default
(`192.168.4.1`) is an access-point address and will not work for a station.

Every 5 seconds the sketch prints a packet count:

```
rx 150 ok=150 crc=0 ver=0 stale=0 | tx 100
```

`ok` should climb at the laptop's command rate. `crc` and `ver` staying at zero
is what tells you `protocol.h` and `app/protocol.py` agree; a rising `ver`
means they have drifted and you should re-copy the sketch.

---

## If the config changes

`config.json` is compiled into the firmware defaults, so after editing it:

```
python tools/gen_config_header.py     # regenerate config_defaults.h
python tools/gen_arduino_sketch.py    # copy it into the sketch
```

then re-upload. The laptop picks up `config.json` changes live; the firmware
does not.

---

## Range sensor

`hal_range_read_mm()` is a stub that honestly reports "no reading", which
`safety.c` treats as *unknown* and never as *clear* — it holds the previous
trip state rather than re-enabling forward motion on a failed sensor.

To fit the real thing at bring-up step 9: install the **SparkFun VL53L1X 4m
Laser Distance Sensor** library, then wire its `begin()`/`startRanging()` into
`hal_range_init()` and `getDistance()` into `hal_range_read_mm()`. Both places
are marked. Then run `python -m app.teleop --drill range`.

---

## Known Arduino-specific notes

- **`micros()` is not used** anywhere, deliberately. It is 32-bit and wraps
  every 71 minutes; the tilt task compares absolute microsecond deadlines, so a
  wrap would freeze the stepper until the counter came round — a fault that
  only appears after an hour, which is the worst kind to debug in a field.
  `esp_timer_get_time()` is 64-bit.
- **`loop()` does almost nothing.** All five tasks are pinned by
  `rover_start_tasks()`. Arduino's `loopTask` would otherwise busy-wait on
  core 1 against the control tasks, so it sleeps.
- **Core 2.x vs 3.x**: `ledcSetup`/`ledcAttachPin`/`ledcWrite(channel, …)`
  became `ledcAttach`/`ledcWrite(pin, …)`. Both are handled behind
  `ESP_ARDUINO_VERSION_MAJOR`, and `make -C firmware_arduino/test` compiles
  against both.
