# CLAUDE.md

jradio-bt — the Bluetooth audio module beside jRadio (`../jradio`): an ESP32
(classic, WROOM-32) running ESP-IDF 5.5.x that does A2DP sink and source,
AVRCP metadata with cover art, and takes orders from the jRadio ESP32-S3 over
one UART. It shares the S3's I2S bus with the PCM5102 DAC; who clocks the bus
depends on the mode (see README.md (README.en.md in English) and the design notes in
`../jradio/.scratch/bluetooth/jradio-bt-proposal.md`).

## Commands

```bash
source <idf-path>/export.sh
idf.py build                         # target esp32 comes from sdkconfig.defaults
bash tools/idf.sh build              # the same with ESP-IDF found and activated (VS Code tasks)
idf.py -p /dev/ttyUSB0 flash monitor
bash tests/run_host_tests.sh         # protocol tests: gcc + sanitizers, no IDF
tools/jbt.py -p /dev/ttyUSB1 ping    # talk to the module from the PC
```

## Layout and conventions

- `components/jbt_proto/` — the wire protocol, pure C, no ESP-IDF. The same
  file is compiled into jRadio's firmware; a change here is a change on both
  sides and belongs in both repositories. Every message and field is
  documented in `jbt_proto.h`, nowhere else.
- `components/jbt_link/` — the UART: receive task, decoder, sends, log mirror.
- `components/bt_stack/` — controller + Bluedroid lifetime and the device
  name; profiles go in their own components on top.
- `main/` — the dispatcher (`main.c`) and the state model (`module_state.c`).
- `tools/jbt.py` — the PC side of the protocol, independent of the C; its
  selftest and the C test pin one shared wire vector.
- Never enable Wi-Fi on this chip: it shares the radio with Bluetooth and
  coexistence is where A2DP dropouts come from.
- Same style as jRadio: comments say why, not what; commits `feat:`/`fix:`
  with a body naming the problem; code and logs in English.
- `feature/esp32-bt-module` is an earlier, unmerged attempt (I2C control);
  its `pcm_ring`/`audio_drift` may be worth lifting later.
