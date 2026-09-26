# jradio-bt

*Читать по-русски: [README.md](README.md).*

The Bluetooth module for [jRadio](https://github.com/jmper-ha/jradio). The
ESP32-S3 that runs jRadio has no Bluetooth Classic - only BLE - and audio
(A2DP) travels over Classic. So Bluetooth here is a second ESP32, a plain
WROOM-32 with this firmware. It sits on the same I2S bus as jRadio's DAC and
takes its orders over a UART.

Both directions:

- **receiving** - a phone connects to the device as to a speaker: sound,
  title, performer, album, cover art, position, volume both ways, pause and
  skipping from the device's knob;
- **sending** - whatever the device plays (radio, files, Yandex, DLNA) goes to
  a Bluetooth speaker or headphones: their buttons do pause, next and
  previous station, and the volume wheel works.

What the device and the web page show meanwhile, and how to use it, is in
[jRadio's documentation](https://github.com/jmper-ha/jradio/blob/main/doc/usage.en.md#bluetooth);
how to wire it, in the [hardware notes](https://github.com/jmper-ha/jradio/blob/main/doc/hardware.en.md#bluetooth-the-jradio-bt-module).

## Wires

| Module (WROOM-32) | jRadio (ESP32-S3) | What |
|---|---|---|
| GPIO 17 (TX) | GPIO 14 | UART 921600 8N1 |
| GPIO 16 (RX) | GPIO 13 | |
| GPIO 26 | GPIO 18 (BCLK) | the shared I2S bus, 33-47 ohm in series at each end |
| GPIO 25 | GPIO 17 (LRCK) | |
| GPIO 22 | GPIO 16 (DOUT) | the same wire goes on to the DAC's DIN |
| GND | GND | |

Pins are changed in `idf.py menuconfig` -> **jradio-bt**. The module never
takes the bus on its own: its three I2S pins are inputs until the host sends
`SET_MODE sink`, and the host sends that only after letting go of its own. When
sending, the S3 stays the master and the module only listens.

The module's antenna next to the S3's disturbs Wi-Fi exactly in the seconds
the module transmits on every channel in turn: a speaker scan, and calling a
speaker that is switched off. Hence the scan lasts five seconds and runs only
on request, and a speaker is called three times and then left to call by
itself. Ordinary audio does not disturb Wi-Fi - adaptive frequency hopping
stays off its channel.

## Flashing

**From the browser, no ESP-IDF:** [jmper-ha.github.io/jradio-bt](https://jmper-ha.github.io/jradio-bt/) -
plug the module in over USB, press the button, pick the port. Chrome or Edge on
a computer. The page is built automatically from a `v*` tag and from every push to
`main` (`.github/workflows/pages.yml`): the firmware is compiled in the ESP-IDF
5.5.5 image, and the three `.bin` files with their manifest go to GitHub Pages;
esptool-js in the browser does the flashing.

**From source:** one build for every board - the module has no screen.
ESP-IDF 5.5.x, the same as jRadio's; how to install it is in
[jRadio's guide](https://github.com/jmper-ha/jradio/blob/main/doc/toolchain.en.md).

In VS Code with the ESP-IDF extension: open the folder, Terminal -> Run Task...
-> **ESP-IDF: Flash** (or "Build, Flash & Monitor"). The tasks call the same
wrappers jRadio has - `tools/idf.sh`, and `tools/idf.ps1` on Windows - which
find the installed ESP-IDF and the port themselves; with two boards attached,
name the module's port in `ESPPORT`. The `esp32` target is already set in
`sdkconfig.defaults`.

From a terminal:

```bash
source <esp-idf>/export.sh
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor    # the module's USB console port
```

The version is in `main/version.h` and in the git tag of the same number; the
exact commit is printed at boot (`jradio-bt 1.0.0 (v1.0.0), protocol 1`), and
the host shows the number on its About page.

## How it is built

- `components/jbt_proto/` - the UART protocol: SLIP framing, a
  `type flags seq len` header, CRC-16/CCITT, strings as TLV. One file for both
  sides: jRadio keeps a copy, and its tests check the two are identical byte
  for byte. Every message is documented in `jbt_proto.h` and nowhere else.
- `components/jbt_link/` - the UART receive task, sending, the log mirror
  (whatever the module prints on its console the host sees as
  `bt_link: bt: ...`).
- `components/bt_stack/` - the controller and Bluedroid, the device name.
- `components/a2dp_sink/` - receiving: A2DP, AVRCP metadata, cover art over
  BIP, volume; `components/audio_out/` - the I2S master with its ring buffer.
- `components/a2dp_source/` - sending: scan, connect, AVRCP both ways (volume
  to the speaker, keys from it); `components/audio_in/` - the I2S slave,
  resampling whatever rate the host clocks to 44.1 kHz.
- `main/` - the message dispatcher and the role switch (Bluedroid does one
  at a time), the phone and the speaker remembered in NVS.

Bluedroid's traps, which took the most time, are explained in comments where
they bit - `a2dp_source.c` (the key filter that rejects the whole set over
one code) and `main.c` (the name before the stack is up, the I2S port across a
role switch).

## Trying it without the S3

A USB-UART on pins 16/17 (not the module's USB console), a phone at hand,
`pyserial`:

```bash
tools/jbt.py -p /dev/ttyUSB1 ping          # PONG protocol=1 firmware=1.0.0
tools/jbt.py -p /dev/ttyUSB1 mode sink     # MODE_ACK mode=sink result=ok
tools/jbt.py -p /dev/ttyUSB1 pairing on    # the phone sees the module
tools/jbt.py -p /dev/ttyUSB1 monitor       # STATUS, TRACK, POSITION, VOLUME...
tools/jbt.py -p /dev/ttyUSB1 mode source
tools/jbt.py -p /dev/ttyUSB1 scan          # speakers around
tools/jbt.py -p /dev/ttyUSB1 connect AA:BB:CC:DD:EE:FF
```

In `sink` mode the sound is already on pins 25/26/22 - any I2S DAC can be
attached. `tools/jbt.py` is a second, independent implementation of the
protocol; it and the firmware are pinned to one test vector.

The protocol tests need no ESP-IDF: `bash tests/run_host_tests.sh` (gcc with
sanitizers).

## History

The branch `feature/esp32-bt-module` holds an August attempt with control over
I2C - not merged, not needed.
