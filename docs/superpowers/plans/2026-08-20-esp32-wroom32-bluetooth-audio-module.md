# ESP32-WROOM-32 Bluetooth Audio Module Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Создать прошивку ESP32-WROOM-32, переключаемую внешним I2C-master между A2DP Sink и A2DP Source, с I2S-аудио, AVRCP 1.6, Cover Art, pairing, reconnect и сервисной UART-консолью.

**Architecture:** Один образ ESP-IDF содержит взаимоисключающие runtime-роли Sink/Source. `mode_manager` последовательно владеет сменой Bluetooth-профилей и I2S-роли; callbacks только передают данные в статические очереди и owner tasks. Управление идёт через версионированный I2C mailbox, аудио — через DMA-backed PCM FIFO, а Cover Art — потоком блоков без полного изображения в RAM.

**Tech Stack:** ESP-IDF v6.0.2, C17, Bluedroid Bluetooth Classic, A2DP SBC external-codec path, AVRCP 1.6/BIP/OBEX, new I2S channel driver, new I2C slave driver, FreeRTOS, NVS, ESP-IDF console/REPL, host `cc` tests и ESP-IDF Unity/HIL tests.

## Global Constraints

- Target только `esp32` на ESP32-WROOM-32 с 4 МБ flash и без PSRAM.
- ESP-IDF закреплён на exact tag `v6.0.2`; `master`, `latest`, ESP-ADF и сторонние Arduino-библиотеки запрещены.
- Первая версия не содержит OTA, Wi-Fi, BLE, AAC, одновременные Source+Sink или преобразование 22,05 кГц.
- Bluetooth-аудио использует SBC, stereo PCM S16, Philips I2S, два 32-битных слота, частоты 32/44,1/48 кГц.
- Sink: внешний контроллер — I2S master, ESP32 — I2S slave TX. Source: ESP32 — I2S master RX, внешний контроллер — slave.
- GPIO задаются Kconfig; base defaults равны `-1`, поэтому без board config firmware загружается в безопасный console-only error state.
- I2C: ESP32 slave, default address `0x2A`, до 400 кГц, без IRQ, polling 20–50 мс, CRC16 и подтверждаемая очередь событий.
- UART0 сохраняется для flash/monitor и сервисной консоли.
- В callbacks/ISR запрещены blocking calls, heap allocation, codec work и разбор command payload.
- PCM work buffers и Cover Art block pool выделяются один раз при init. Единственное runtime-выделение — `esp_a2d_audio_buff_alloc()` в отдельной Source sender task, потому что успешный `esp_a2d_source_audio_data_send()` передаёт ownership Bluedroid; callbacks, DMA и codec processing malloc/free не выполняют.
- Каждый этап заканчивается тестом, `scripts/build.sh`, `git diff --check` и отдельным commit.

**Design spec:** `docs/superpowers/specs/2026-08-20-esp32-wroom32-bluetooth-audio-module-design.md`

---

## Planned File Structure

```text
CMakeLists.txt
Kconfig.projbuild
sdkconfig.defaults
partitions.csv
.gitignore
README.md
scripts/preflight.sh
scripts/build.sh
scripts/flash.sh
scripts/monitor.sh
scripts/test-host.sh
main/CMakeLists.txt
main/app_main.c
components/jradio_common/include/jradio_types.h
components/control_protocol/{CMakeLists.txt,control_protocol.c,event_queue.c,include/*.h}
components/settings/{CMakeLists.txt,settings.c,settings_nvs.c,include/settings.h}
components/mode_manager/{CMakeLists.txt,mode_manager.c,include/mode_manager.h}
components/audio/{CMakeLists.txt,pcm_ring.c,audio_drift.c,audio_i2s.c,include/*.h}
components/sbc_codec/{CMakeLists.txt,sbc_codec.c,include/sbc_codec.h}
components/bluetooth/{CMakeLists.txt,bt_core.c,bt_gap.c,a2dp_sink.c,a2dp_source.c,avrcp_bridge.c,cover_art.c,include/*.h}
components/control_i2c/{CMakeLists.txt,control_i2c.c,control_service.c,include/*.h}
components/diagnostics/{CMakeLists.txt,diagnostics.c,console.c,include/*.h}
tests/host/{test_protocol.c,test_event_queue.c,test_mode_manager.c,test_pcm_ring.c,test_audio_drift.c,test_cover_art.c}
test_apps/unit/{CMakeLists.txt,sdkconfig.defaults,main/CMakeLists.txt,main/test_runner.c,main/test_*.c}
docs/protocol/i2c-control-protocol.md
docs/validation/hil-matrix.md
docs/validation/soak-test-log.md
```

## Task 1: Reproducible ESP-IDF Project and Toolchain Gate

**Files:**
- Create: `.gitignore`, `CMakeLists.txt`, `Kconfig.projbuild`, `sdkconfig.defaults`, `partitions.csv`, `README.md`
- Create: `scripts/preflight.sh`, `scripts/build.sh`, `scripts/flash.sh`, `scripts/monitor.sh`, `scripts/test-host.sh`
- Create: `main/CMakeLists.txt`, `main/app_main.c`

**Interfaces:**
- Produces: `scripts/preflight.sh` that accepts only `ESP-IDF v6.0.2` and target `esp32`.
- Produces: buildable firmware that remains in a safe state when GPIO values are `-1`.

- [ ] **Step 1: Initialize version control and add the project ignore policy**

Run:

```bash
git init --initial-branch=main
```

Create `.gitignore` containing `build/`, `build-host/`, `sdkconfig`, `sdkconfig.old`, `.pytest_cache/`, `managed_components/` and monitor logs, while keeping `sdkconfig.defaults` tracked.

- [ ] **Step 2: Write the failing preflight check**

Create `scripts/preflight.sh` with strict shell mode and these checks:

```bash
#!/usr/bin/env bash
set -euo pipefail
command -v idf.py >/dev/null
actual="$(idf.py --version)"
test "$actual" = "ESP-IDF v6.0.2"
test "$(idf.py --list-targets | tr ' ' '\n' | grep -x esp32)" = "esp32"
```

Run: `bash scripts/preflight.sh`
Expected now: FAIL because the current shell has no `idf.py`; during execution, source the v6.0.2 `export.sh` and rerun until PASS. Do not silently use the locally installed v5.5.x toolchain.

- [ ] **Step 3: Add the root ESP-IDF project and exact flash layout**

Use:

```cmake
cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(jradio_bt)
```

Create `partitions.csv`:

```csv
# Name,   Type, SubType, Offset,   Size,     Flags
nvs,      data, nvs,     0x9000,   0x6000,
phy_init, data, phy,     0xf000,   0x1000,
factory,  app,  factory, 0x10000,  0x3F0000,
```

Set the Bluetooth Classic/A2DP/external codec/Cover Art/custom partition options in `sdkconfig.defaults`, including `CONFIG_BT_ENABLED=y`, `CONFIG_BT_BLUEDROID_ENABLED=y`, `CONFIG_BT_CLASSIC_ENABLED=y`, `CONFIG_BT_A2DP_ENABLE=y`, `CONFIG_BT_A2DP_USE_EXTERNAL_CODEC=y`, `CONFIG_BT_AVRCP_CT_COVER_ART_ENABLED=y`, `CONFIG_ESPTOOLPY_FLASHSIZE_4MB=y`, and `CONFIG_PARTITION_TABLE_CUSTOM=y`.

- [ ] **Step 4: Add configurable board pins with invalid safe defaults**

`Kconfig.projbuild` defines signed integer options for `JRADIO_I2C_SDA_GPIO`, `JRADIO_I2C_SCL_GPIO`, `JRADIO_I2S_BCLK_GPIO`, `JRADIO_I2S_WS_GPIO`, `JRADIO_I2S_DIN_GPIO`, `JRADIO_I2S_DOUT_GPIO`, all defaulting to `-1`, plus I2C address default `0x2A`.

`app_main.c` must log firmware/IDF/chip/reset information and return to console-only mode when required pins for the selected mode are invalid.

- [ ] **Step 5: Add wrappers and build**

`scripts/build.sh` runs `preflight.sh`, `idf.py set-target esp32` only when no build directory exists, then `idf.py build`, `idf.py size`, and `idf.py size-components`. Flash/monitor wrappers require an explicit port argument and use 115200 baud for monitor.

Run: `bash scripts/build.sh`
Expected: PASS with a bootable minimal image and no partition overflow.

- [ ] **Step 6: Commit**

```bash
git add .gitignore CMakeLists.txt Kconfig.projbuild sdkconfig.defaults partitions.csv README.md scripts main
git commit -m "build: bootstrap ESP-IDF v6.0.2 project"
```

## Task 2: Shared Types, Wire Framing, and Event Queue

**Files:**
- Create: `components/jradio_common/include/jradio_types.h`, `components/jradio_common/CMakeLists.txt`
- Create: `components/control_protocol/include/control_protocol.h`, `components/control_protocol/include/event_queue.h`
- Create: `components/control_protocol/control_protocol.c`, `components/control_protocol/event_queue.c`, `components/control_protocol/CMakeLists.txt`
- Create: `tests/host/test_protocol.c`, `tests/host/test_event_queue.c`
- Modify: `scripts/test-host.sh`

**Interfaces:**
- Produces: `jr_mode_t`, `jr_state_t`, `jr_audio_format_t`, `jr_bt_addr_t`, `jr_error_t`.
- Produces: `jr_frame_encode()`, `jr_frame_decode()`, `jr_crc16_ccitt()`.
- Produces: `jr_event_queue_init()`, `jr_event_queue_push()`, `jr_event_queue_peek()`, `jr_event_queue_ack()`.

- [ ] **Step 1: Define exact wire-independent shared types**

Use fixed-width fields and explicit enums; never serialize C structs directly:

```c
typedef enum { JR_MODE_SINK = 0, JR_MODE_SOURCE = 1 } jr_mode_t;
typedef enum {
    JR_STATE_BOOT, JR_STATE_IDLE, JR_STATE_MODE_STARTING,
    JR_STATE_DISCOVERABLE, JR_STATE_SCANNING, JR_STATE_CONNECTING,
    JR_STATE_CONNECTED, JR_STATE_CLOCK_WAIT, JR_STATE_STREAMING,
    JR_STATE_DISCONNECTING, JR_STATE_MODE_SWITCHING,
    JR_STATE_RECOVERING, JR_STATE_ERROR
} jr_state_t;
typedef struct {
    uint32_t sample_rate_hz;
    uint8_t channels;
    uint8_t sample_bits;
    uint8_t slot_bits;
} jr_audio_format_t;
```

- [ ] **Step 2: Write failing frame tests**

Test golden bytes for magic `0x4A52`, protocol version `1`, little-endian `opcode`, `sequence`, `payload_length`, payload, and CRC16-CCITT with polynomial `0x1021`, initial value `0xFFFF`. Assert rejection of wrong magic, version, length, CRC, payload over 512 bytes, and truncated input.

```c
assert(jr_frame_decode(golden, sizeof(golden), &frame) == JR_OK);
golden[sizeof(golden) - 1] ^= 1;
assert(jr_frame_decode(golden, sizeof(golden), &frame) == JR_ERR_CRC);
```

Run: `bash scripts/test-host.sh protocol`
Expected: FAIL because codec functions are missing.

- [ ] **Step 3: Implement framing without packed structs**

Use byte-wise helpers `put_u16_le`, `put_u32_le`, `get_u16_le`, `get_u32_le`; validate every length before reading. Add `_Static_assert(JR_PROTO_MAX_PAYLOAD == 512, ...)`.

Run: `bash scripts/test-host.sh protocol`
Expected: PASS.

- [ ] **Step 4: Write failing event queue tests**

Use capacity 32. Verify monotonic IDs, FIFO order, ACK only for the head ID, idempotent repeated ACK, sticky overflow, loss counter, and that payload data is copied rather than borrowed.

- [ ] **Step 5: Implement the fixed-capacity queue**

The queue owns `jr_event_t slots[32]`, `head`, `count`, `next_id`, `lost`, and `overflow_sticky`. `push` fails with `JR_ERR_FULL` and increments `lost`; it never overwrites an unacknowledged event.

Run: `bash scripts/test-host.sh`
Expected: all host tests PASS.

- [ ] **Step 6: Commit**

```bash
git add components/jradio_common components/control_protocol tests/host scripts/test-host.sh
git commit -m "feat: add versioned control framing and event queue"
```

## Task 3: Persistent Settings and Bonding Policy

**Files:**
- Create: `components/settings/include/settings.h`, `components/settings/settings.c`, `components/settings/settings_nvs.c`, `components/settings/CMakeLists.txt`
- Create: `tests/host/test_settings.c`

**Interfaces:**
- Produces: `jr_settings_defaults()`, `jr_settings_validate()`, `jr_settings_load()`, `jr_settings_save()`.
- Produces: separate `last_sink_peer`, `last_source_peer`, auto-reconnect flags, local name, last confirmed mode.

- [ ] **Step 1: Write failing validation and migration tests**

Define schema version `1`, UTF-8 local name maximum 63 bytes, and explicit validity flags for peer addresses. Verify erased NVS maps to defaults, invalid mode is rejected, version mismatch returns `JR_ERR_SCHEMA`, and Sink/Source addresses never overwrite each other.

- [ ] **Step 2: Implement the pure settings model**

```c
typedef struct {
    uint16_t schema_version;
    jr_mode_t last_mode;
    bool auto_reconnect_sink;
    bool auto_reconnect_source;
    bool has_last_sink_peer;
    bool has_last_source_peer;
    jr_bt_addr_t last_sink_peer;
    jr_bt_addr_t last_source_peer;
    char local_name[64];
} jr_settings_t;
```

Defaults: Sink mode, both auto-reconnect flags true, name `JRadio BT`, no peer addresses.

- [ ] **Step 3: Add the NVS adapter**

Use namespace `jradio`, one versioned binary blob plus a CRC32. Write a new blob only when contents changed. On CRC/schema failure, keep the invalid blob for diagnostics, load defaults, and publish a settings error rather than erasing all Bluetooth NVS.

- [ ] **Step 4: Add on-target persistence test**

The Unity test writes a non-default settings object, deinitializes/reinitializes NVS, verifies round-trip, then restores defaults. It must not erase the global NVS partition.

Run: `bash scripts/test-host.sh settings && bash scripts/build.sh`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add components/settings tests/host/test_settings.c test_apps
git commit -m "feat: persist mode and peer settings in NVS"
```

## Task 4: Deterministic Mode State Machine

**Files:**
- Create: `components/mode_manager/include/mode_manager.h`, `components/mode_manager/mode_manager.c`, `components/mode_manager/CMakeLists.txt`
- Create: `tests/host/test_mode_manager.c`

**Interfaces:**
- Consumes: `jr_mode_t`, `jr_state_t` and async subsystem completion events.
- Produces: `jr_mode_manager_request_mode()`, `jr_mode_manager_post()`, `jr_mode_manager_snapshot()`.
- Produces: `jr_mode_ops_t` dependency table for audio/AVRCP/A2DP/GAP/Bluetooth restart calls.

- [ ] **Step 1: Write the transition table as failing tests**

Test Sink→Source sequence exactly: reject new media work, cancel Cover Art, mute, stop I2S, disconnect, deinit AVRCP, deinit A2DP, configure Source I2S, init AVRCP, init A2DP, reconnect/scan, publish `MODE_CHANGED`.

Also test duplicate requests, a new request during transition, callbacks with stale generation, timeout to Bluetooth restart, and second timeout to `JR_MODE_ACTION_RESTART_MCU`.

- [ ] **Step 2: Define the operation boundary**

```c
typedef struct {
    jr_error_t (*cancel_cover_art)(void *ctx);
    jr_error_t (*audio_stop)(void *ctx);
    jr_error_t (*bt_disconnect)(void *ctx);
    jr_error_t (*avrcp_deinit)(void *ctx, jr_mode_t old_mode);
    jr_error_t (*a2dp_deinit)(void *ctx, jr_mode_t old_mode);
    jr_error_t (*audio_configure)(void *ctx, jr_mode_t new_mode);
    jr_error_t (*avrcp_init)(void *ctx, jr_mode_t new_mode);
    jr_error_t (*a2dp_init)(void *ctx, jr_mode_t new_mode);
    jr_error_t (*bt_stack_restart)(void *ctx);
    void (*restart_mcu)(void *ctx);
} jr_mode_ops_t;
```

- [ ] **Step 3: Implement a table-driven nonblocking state machine**

No operation waits for completion. Each issued action records `generation`, expected completion event and deadline. `jr_mode_manager_post()` advances only on matching generation/event. A periodic owner task calls `jr_mode_manager_tick(now_ms)` for timeouts.

- [ ] **Step 4: Run tests and build**

Run: `bash scripts/test-host.sh mode_manager && bash scripts/build.sh`
Expected: all transition tests PASS.

- [ ] **Step 5: Commit**

```bash
git add components/mode_manager tests/host/test_mode_manager.c
git commit -m "feat: add deterministic Bluetooth mode state machine"
```

## Task 5: PCM Ring and Sink Clock-Drift Compensation

**Files:**
- Create: `components/audio/include/pcm_ring.h`, `components/audio/include/audio_drift.h`
- Create: `components/audio/pcm_ring.c`, `components/audio/audio_drift.c`, `components/audio/CMakeLists.txt`
- Create: `tests/host/test_pcm_ring.c`, `tests/host/test_audio_drift.c`

**Interfaces:**
- Produces: interleaved stereo-frame ring APIs using caller-owned static storage.
- Produces: `jr_drift_update()` and `jr_drift_process_s16_stereo()`.

- [ ] **Step 1: Write ring-buffer boundary tests**

Verify wraparound, exact full/empty distinction, partial writes/reads, frame order, high-water mark, underrun and overrun counters. Capacity is expressed in stereo frames, never bytes.

- [ ] **Step 2: Implement the ring**

```c
typedef struct {
    int16_t *samples;
    size_t capacity_frames;
    size_t read_frame;
    size_t write_frame;
    size_t count_frames;
    uint32_t underruns;
    uint32_t overruns;
} jr_pcm_ring_t;
```

The owner task serializes normal access; short cross-context index snapshots use a bounded critical section, not a mutex around DMA calls.

- [ ] **Step 3: Write drift-controller tests**

Feed simulated external clocks at nominal, +250 ppm and -250 ppm for eight virtual hours. Require bounded FIFO level, ratio within configured ±1000 ppm, monotonic phase, and no out-of-range sample values. Verify ramp-to-zero on forced underrun and crossfade path on forced overrun.

- [ ] **Step 4: Implement PI control and interpolation**

Use target fill 50%, update period 20 ms, integral clamp, ratio clamp `[0.999, 1.001]`, and persistent Q32.32 fractional phase. Linear interpolation operates on left/right independently and saturates to `int16_t`.

- [ ] **Step 5: Run analysis tests**

Run: `bash scripts/test-host.sh pcm_ring audio_drift`
Expected: PASS; test output records max fill deviation and peak ratio.

- [ ] **Step 6: Commit**

```bash
git add components/audio tests/host/test_pcm_ring.c tests/host/test_audio_drift.c
git commit -m "feat: add PCM buffering and clock-drift compensation"
```

## Task 6: I2S Driver for Both Clock Roles

**Files:**
- Create: `components/audio/include/audio_i2s.h`, `components/audio/audio_i2s.c`
- Create: `test_apps/unit/main/test_audio_i2s.c`
- Modify: `components/audio/CMakeLists.txt`, `test_apps/unit/main/CMakeLists.txt`

**Interfaces:**
- Produces: `jr_audio_i2s_configure(mode, format)`, `jr_audio_i2s_start()`, `jr_audio_i2s_stop()`.
- Produces: bounded `jr_audio_i2s_read()` and `jr_audio_i2s_write()` with byte counts and timeouts.

- [ ] **Step 1: Write failing configuration tests around an injected driver API**

Assert Sink creates TX with `I2S_ROLE_SLAVE`; Source creates RX with `I2S_ROLE_MASTER` and `I2S_CLK_SRC_APLL`. Both must set 16-bit data width, 32-bit slot width, stereo both-slot mask and Philips one-bit shift. Reject 16/22,05 kHz and missing required GPIOs.

- [ ] **Step 2: Implement lifecycle with the new channel driver**

Use `i2s_new_channel()`, `i2s_channel_init_std_mode()`, `i2s_channel_enable()`, `i2s_channel_disable()`, and `i2s_del_channel()`. Convert between DMA `int32_t` slots and internal packed stereo `int16_t` frames in fixed work buffers. Never delete a channel while enabled.

- [ ] **Step 3: Add external-clock loss handling**

Sink writes use finite timeouts. Consecutive timeouts transition audio status to `CLOCK_LOST`, stop channel ownership cleanly and notify `mode_manager`; they do not loop forever or reset from callback context.

- [ ] **Step 4: Run on-target loopback**

Wire DIN↔DOUT and shared BCLK/WS for the test fixture. For 32/44,1/48 kHz, transmit a counter/ramp and verify sample alignment, left/right order and 32-bit slot packing with zero mismatches over one million frames.

Run: `idf.py -C test_apps/unit build flash monitor`
Expected: all `audio_i2s` Unity cases PASS.

- [ ] **Step 5: Commit**

```bash
git add components/audio test_apps/unit/main/test_audio_i2s.c
git commit -m "feat: support master and slave I2S audio paths"
```

## Task 7: Application-Managed SBC Codec Wrapper

**Files:**
- Create: `components/sbc_codec/include/sbc_codec.h`, `components/sbc_codec/sbc_codec.c`, `components/sbc_codec/CMakeLists.txt`
- Create: `test_apps/unit/main/test_sbc_codec.c`
- Create: `docs/dependencies.md`

**Interfaces:**
- Produces opaque `jr_sbc_encoder_t` and `jr_sbc_decoder_t` allocated only at init.
- Produces `jr_sbc_encoder_configure()`, `jr_sbc_encode()`, `jr_sbc_decoder_reset()`, `jr_sbc_decode()`.

- [ ] **Step 1: Compile the pinned ESP-IDF SBC sources as the application codec**

With `CONFIG_BT_A2DP_USE_EXTERNAL_CODEC=y`, ESP-IDF intentionally excludes SBC implementation objects from `bt`. In `components/sbc_codec/CMakeLists.txt`, set `SBC_ROOT` to `$ENV{IDF_PATH}/components/bt/host/bluedroid/external/sbc`, add its encoder/decoder include directories privately, and explicitly compile these v6.0.2 sources without `GLOB`:

```cmake
set(SBC_DECODER_SRCS
    ${SBC_ROOT}/decoder/srce/alloc.c
    ${SBC_ROOT}/decoder/srce/bitalloc-sbc.c
    ${SBC_ROOT}/decoder/srce/bitalloc.c
    ${SBC_ROOT}/decoder/srce/bitstream-decode.c
    ${SBC_ROOT}/decoder/srce/decoder-oina.c
    ${SBC_ROOT}/decoder/srce/decoder-private.c
    ${SBC_ROOT}/decoder/srce/decoder-sbc.c
    ${SBC_ROOT}/decoder/srce/dequant.c
    ${SBC_ROOT}/decoder/srce/framing-sbc.c
    ${SBC_ROOT}/decoder/srce/framing.c
    ${SBC_ROOT}/decoder/srce/oi_codec_version.c
    ${SBC_ROOT}/decoder/srce/synthesis-8-generated.c
    ${SBC_ROOT}/decoder/srce/synthesis-dct8.c
    ${SBC_ROOT}/decoder/srce/synthesis-sbc.c)
set(SBC_ENCODER_SRCS
    ${SBC_ROOT}/encoder/srce/sbc_analysis.c
    ${SBC_ROOT}/encoder/srce/sbc_dct.c
    ${SBC_ROOT}/encoder/srce/sbc_dct_coeffs.c
    ${SBC_ROOT}/encoder/srce/sbc_enc_bit_alloc_mono.c
    ${SBC_ROOT}/encoder/srce/sbc_enc_bit_alloc_ste.c
    ${SBC_ROOT}/encoder/srce/sbc_enc_coeffs.c
    ${SBC_ROOT}/encoder/srce/sbc_encoder.c
    ${SBC_ROOT}/encoder/srce/sbc_packing.c)
```

Compile smoke calls to `SBC_Encoder_Init()` and `OI_CODEC_SBC_DecoderReset()` against exact IDF v6.0.2. Run: `idf.py -C test_apps/unit build`. Expected: wrapper symbols resolve with no duplicate definitions.

- [ ] **Step 2: Write failing round-trip tests**

Generate stereo PCM sine waves in the test for each supported rate. Encode SBC with 16 blocks, 8 subbands, joint stereo, loudness allocation and legal bitpool, then decode every frame. Assert sample count, channel count, no decoder errors, and normalized correlation above 0.95 after codec delay.

- [ ] **Step 3: Implement the wrapper**

Hide all private IDF SBC types inside `sbc_codec.c`. Translate `esp_a2d_cie_sbc_t` to wrapper config. Validate bitpool, frame length and output capacity before every call. No allocation occurs in encode/decode.

- [ ] **Step 4: Document the dependency**

`docs/dependencies.md` records tag `v6.0.2`, the private header paths, APIs used, Apache-2.0 notices inherited from ESP-IDF, and the required revalidation when changing IDF.

- [ ] **Step 5: Run codec tests and size report**

Run: `idf.py -C test_apps/unit build flash monitor` and `bash scripts/build.sh`
Expected: all codec cases PASS and RAM/flash delta is recorded in the commit message.

- [ ] **Step 6: Commit**

```bash
git add components/sbc_codec test_apps/unit/main/test_sbc_codec.c docs/dependencies.md
git commit -m "feat: wrap pinned ESP-IDF SBC codec"
```

## Task 8: Bluetooth Core, GAP Discovery, Pairing, and Reconnect

**Files:**
- Create: `components/bluetooth/include/bt_core.h`, `components/bluetooth/include/bt_gap.h`
- Create: `components/bluetooth/bt_core.c`, `components/bluetooth/bt_gap.c`, `components/bluetooth/CMakeLists.txt`
- Create: `test_apps/unit/main/test_bt_gap.c`

**Interfaces:**
- Produces: `jr_bt_core_init()`, `jr_bt_core_restart()`, `jr_bt_core_deinit()`.
- Produces: `jr_bt_gap_start_discovery()`, `jr_bt_gap_stop_discovery()`, `jr_bt_gap_set_connectable()`, `jr_bt_gap_open_pairing_window()`, `jr_bt_gap_pairing_reply()` and normalized `jr_bt_event_t` callback to the owner task.

- [ ] **Step 1: Write callback-normalization tests**

Feed synthetic GAP events for discovery result, discovery complete, auth complete/fail, PIN request, numeric comparison and disconnect. Assert deep-copy of name/EIR data before callback return, address preservation and correct normalized event types.

- [ ] **Step 2: Implement deterministic stack initialization**

Order: NVS ready → `esp_bt_controller_init()` → `esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)` → `esp_bluedroid_init_with_cfg()` → `esp_bluedroid_enable()` → GAP callback/security params. Do not release Classic BT memory. Register one application dispatch queue before profiles.

- [ ] **Step 3: Implement discovery and security policy**

Sink mode becomes connectable/discoverable only during an explicit pairing window. Source discovery publishes every candidate without filtering by a hard-coded name. SSP defaults to `Just Works`; PIN/numeric comparison operations wait for an I2C response with a finite deadline.

- [ ] **Step 4: Implement reconnect backoff**

For the saved peer of the active mode, use delays 1, 2, 4, 8 and 16 seconds, then stop and publish `RECONNECT_EXHAUSTED`. A user command resets the attempt count.

- [ ] **Step 5: Hardware smoke test**

Verify scan results from at least one phone and one speaker, pairing window enforcement, bond persistence across restart, and separate last peers for both modes.

- [ ] **Step 6: Commit**

```bash
git add components/bluetooth test_apps/unit/main/test_bt_gap.c
git commit -m "feat: add Classic Bluetooth discovery and pairing"
```

## Task 9: A2DP Sink Audio Path

**Files:**
- Create: `components/bluetooth/include/a2dp_sink.h`, `components/bluetooth/a2dp_sink.c`
- Create: `test_apps/unit/main/test_a2dp_sink.c`
- Modify: `components/bluetooth/CMakeLists.txt`

**Interfaces:**
- Consumes: SBC decoder, PCM ring, drift processor, I2S Sink/slave TX.
- Produces: `jr_a2dp_sink_init()`, `jr_a2dp_sink_connect()`, `jr_a2dp_sink_disconnect()`, `jr_a2dp_sink_deinit()`.

- [ ] **Step 1: Write the profile lifecycle test**

Verify AVRCP init is requested before `esp_a2d_sink_init()`, then SBC SEP registration and `esp_a2d_sink_register_audio_data_callback()`. Assert no connection is accepted before profile-init and SEP-register completion events.

- [ ] **Step 2: Implement Sink callbacks as ownership transfer only**

The A2DP audio callback enqueues the `esp_a2d_audio_buff_t *` into a bounded decoder queue and returns. The decoder task consumes all SBC frames, frees every buffer exactly once with `esp_a2d_audio_buff_free()`, and writes PCM frames to the ring. On queue full, it increments a drop counter, frees the buffer and publishes a quality error.

- [ ] **Step 3: Implement format/clock handshake**

On `ESP_A2D_AUDIO_CFG_EVT`, accept only SBC and 32/44,1/48 kHz stereo-compatible modes, reset the decoder and publish `AUDIO_FORMAT_CHANGED`. Do not start I2S until the matching I2C `AUDIO_CLOCK_READY` sequence arrives.

- [ ] **Step 4: Integrate rendering and delay report**

The audio task drains decoded PCM through `audio_drift` into I2S TX. Report total buffering/render delay through `esp_a2d_sink_set_delay_value()` in 0.1 ms units, never below ESP-IDF's required 120 ms.

- [ ] **Step 5: Run Sink HIL smoke test**

Stream 32, 44,1 and 48 kHz from Android/Windows, remove/restore external BCLK, verify no panic and correct `CLOCK_WAIT`/`STREAMING` events.

- [ ] **Step 6: Commit**

```bash
git add components/bluetooth/a2dp_sink.c components/bluetooth/include/a2dp_sink.h test_apps/unit/main/test_a2dp_sink.c
git commit -m "feat: implement A2DP sink audio pipeline"
```

## Task 10: A2DP Source Audio Path

**Files:**
- Create: `components/bluetooth/include/a2dp_source.h`, `components/bluetooth/a2dp_source.c`
- Create: `test_apps/unit/main/test_a2dp_source.c`

**Interfaces:**
- Consumes: I2S Source/master RX, PCM ring and SBC encoder.
- Produces: `jr_a2dp_source_init()`, `jr_a2dp_source_connect()`, `jr_a2dp_source_disconnect()`, `jr_a2dp_source_deinit()`.

- [ ] **Step 1: Write negotiation tests**

Given synthetic peer capabilities, choose only SBC and prefer 48, then 44,1, then 32 kHz. Reject peers without a common rate. Verify `esp_a2d_source_set_pref_mcc()` occurs after connection capability report and before media start.

- [ ] **Step 2: Implement Source lifecycle and SEP**

Initialize AVRCP first, then A2DP Source and SBC SEP. Record connection handle and audio MTU from `ESP_A2D_CONNECTION_STATE_EVT`; clear them atomically on disconnect.

- [ ] **Step 3: Implement I2S clock and encode task**

Configure ESP32 I2S master at the selected rate, notify the external controller, then read fixed PCM frame groups into static work buffers. Encode into a static packet workspace. The dedicated sender task obtains the required ownership-transfer object with `esp_a2d_audio_buff_alloc()` immediately before send, copies the encoded packet, and sets `number_frame`, `data_len` and timestamp consistently. Track allocation count/failures and stop the stream cleanly after a bounded run of allocation failures.

- [ ] **Step 4: Implement congestion handling**

Call `esp_a2d_source_audio_data_send()`. On `ESP_FAIL` queue-full, retain ownership and retry with bounded task delay; on invalid state/size, stop media and publish a fatal stream error. Never busy-loop in the encoder task.

- [ ] **Step 5: Run Source HIL smoke test**

Connect at least two speaker/headphone models, inject a 1 kHz stereo signal over I2S, verify correct rate and uninterrupted audio for 30 minutes at each supported rate.

- [ ] **Step 6: Commit**

```bash
git add components/bluetooth/a2dp_source.c components/bluetooth/include/a2dp_source.h test_apps/unit/main/test_a2dp_source.c
git commit -m "feat: implement A2DP source audio pipeline"
```

## Task 11: AVRCP Commands, Metadata, and Cover Art Streaming

**Files:**
- Create: `components/bluetooth/include/avrcp_bridge.h`, `components/bluetooth/include/cover_art.h`
- Create: `components/bluetooth/avrcp_bridge.c`, `components/bluetooth/cover_art.c`
- Create: `tests/host/test_cover_art.c`, `test_apps/unit/main/test_avrcp.c`

**Interfaces:**
- Produces AVRCP play/pause/stop/next/previous/absolute-volume commands and normalized metadata events.
- Produces `jr_avrcp_send_command()`, `jr_avrcp_set_absolute_volume()` and normalized metadata events.
- Produces `jr_cover_art_get_properties()`, `jr_cover_art_request()`, `jr_cover_art_cancel()`, `jr_cover_art_peek_chunk()` and `jr_cover_art_ack_chunk()`.

- [ ] **Step 1: Write metadata ownership tests**

Feed metadata callbacks whose backing buffers are overwritten immediately after return. Assert title/artist/album/genre/cover-handle were copied with UTF-8 length limits and that oversized fields produce truncation status rather than overflow.

- [ ] **Step 2: Implement AVRCP lifecycle and notifications**

Initialize CT/TG before A2DP. Register remote capability notifications for play status, track change, play position and volume. Translate every command into pressed and released pass-through calls with a transaction label allocator that does not reuse an outstanding label.

- [ ] **Step 3: Write Cover Art chunk tests**

For a synthetic 1300-byte JPEG event stream, expect chunks `512`, `512`, `276` with stable object ID, offsets `0/512/1024`, individual CRC, final only on the third block, ACK-in-order, cancel cleanup and backpressure when all four pool blocks are occupied.

- [ ] **Step 4: Implement the four-block Cover Art pool**

Enable AVRCP Cover Art CT, connect OBEX with an MTU that fits the bounded callback-copy path, request properties before image data, and split callback payloads into four 512-byte blocks. The I2C master may request original, linked thumbnail or an image descriptor. No image decoding or whole-object allocation is allowed.

- [ ] **Step 5: Add unsupported/error behavior**

Map peer-without-Cover-Art to `JR_ERR_NOT_SUPPORTED`; retain audio/metadata. Timeout, cancel and peer disconnect must release every occupied block and close the Cover Art connection without deinitializing A2DP.

- [ ] **Step 6: Run tests and commit**

Run: `bash scripts/test-host.sh cover_art && idf.py -C test_apps/unit build flash monitor`
Expected: all metadata/chunk tests PASS.

```bash
git add components/bluetooth tests/host/test_cover_art.c test_apps/unit/main/test_avrcp.c
git commit -m "feat: add AVRCP metadata and Cover Art streaming"
```

## Task 12: I2C Slave Transport and Command Service

**Files:**
- Create: `components/control_i2c/include/control_i2c.h`, `components/control_i2c/include/control_service.h`
- Create: `components/control_i2c/control_i2c.c`, `components/control_i2c/control_service.c`, `components/control_i2c/CMakeLists.txt`
- Create: `test_apps/unit/main/test_control_i2c.c`
- Create: `docs/protocol/i2c-control-protocol.md`

**Interfaces:**
- Consumes: frame codec/event queue and all subsystem command interfaces.
- Produces: `jr_control_i2c_init()`, `jr_control_i2c_deinit()`, `jr_control_service_submit_frame()`, `jr_control_service_get_status()` and `jr_control_service_get_response()` with request/response correlation and idempotency cache.

- [ ] **Step 1: Freeze protocol v1 opcodes and payload schemas**

Define explicit numeric opcodes grouped by device, mode, Bluetooth, audio, AVRCP, Cover Art and diagnostics. Document byte offsets, lengths, units, enum values, timeout behavior and examples in `docs/protocol/i2c-control-protocol.md`. Protocol v1 payload maximum is 512 bytes.

- [ ] **Step 2: Write transaction tests**

Test master-write request, repeated-start status read, response read, duplicate sequence, CRC error, unknown opcode, busy response, event peek/ACK, queue overflow and 512-byte Cover Art reads. Simulate polling gaps of 1 second without losing acknowledged semantics.

- [ ] **Step 3: Implement ISR-safe transport**

Configure `i2c_new_slave_device()` with Kconfig pins/address and fixed RX/TX depths. `on_receive` and `on_request` only copy descriptors/notify the owner task with `xQueueSendFromISR()`/`vTaskNotifyGiveFromISR()`. The task performs `i2c_slave_write()` with finite timeout and prebuilt response data.

- [ ] **Step 4: Implement command dispatch and idempotency**

Validate mode/state before executing each opcode. Cache the last completed sequence and encoded response; a duplicate sequence returns the cached response and never repeats connect, factory-reset or mode-switch actions. Long operations return `ACCEPTED` and finish via event IDs.

- [ ] **Step 5: Validate with a real I2C master**

Use the external controller or a second development board at 100 and 400 kHz. Exercise repeated-start, polling, corrupted CRC and deliberate one-second stalls while Bluetooth audio runs.

- [ ] **Step 6: Commit**

```bash
git add components/control_i2c test_apps/unit/main/test_control_i2c.c docs/protocol
git commit -m "feat: expose control mailbox over I2C slave"
```

## Task 13: Application Integration, Recovery, and Service Console

**Files:**
- Create: `components/diagnostics/include/diagnostics.h`, `components/diagnostics/diagnostics.c`, `components/diagnostics/console.c`, `components/diagnostics/CMakeLists.txt`
- Modify: `main/app_main.c`, `main/CMakeLists.txt`
- Create: `test_apps/unit/main/test_integration.c`

**Interfaces:**
- Produces the complete boot sequence and owner tasks.
- Produces console commands `status`, `version`, `mode`, `bt`, `audio`, `i2c`, `heap`, `tasks`, `loglevel`, `reboot`, `factory-reset`.

- [ ] **Step 1: Write boot-order and cleanup tests**

With injected subsystem ops, assert order: NVS → settings → protocol/event queue → diagnostics → I2C → audio buffers → Bluetooth core → persisted mode. Inject failure at every stage and verify reverse-order cleanup plus an actionable error snapshot.

- [ ] **Step 2: Implement task ownership**

Create bounded queues/tasks for mode/control, Bluetooth events, Sink decode/render, Source capture/encode and Cover Art. Start without core affinity; record priorities, stack sizes and queue depths in `diagnostics.c`. Enable watchdog monitoring only after tasks demonstrate normal liveness.

- [ ] **Step 3: Integrate mode switching and escalation**

Wire all completion events to generation-aware `mode_manager`. First timeout restarts controller/Bluedroid and re-registers callbacks/profiles. Second failure persists target mode and reason, flushes logs, then calls `esp_restart()`.

- [ ] **Step 4: Implement ESP-IDF REPL**

Use `esp_console`/linenoise REPL on UART0 with help/history/autocomplete. Commands only enqueue normal application requests; they never call Bluetooth or I2S APIs concurrently from console context. Require typed confirmation `factory-reset YES`.

- [ ] **Step 5: Add diagnostic snapshots**

Expose current mode/state/generation, peer, codec/rate, FIFO level/high-water, drift ratio, underrun/overrun, Cover Art blocks, I2C CRC/loss counts, heap free/min/largest block, task stack watermarks and last reset/recovery reason.

- [ ] **Step 6: Run integration cycle**

Run 100 automated Sink↔Source cycles with mocked peers or controllable test peers. Expected: no stale callback changes state, heap minimum stabilizes after warm-up, every request gets response/event, and failed transitions recover.

- [ ] **Step 7: Commit**

```bash
git add components/diagnostics main test_apps/unit/main/test_integration.c
git commit -m "feat: integrate runtime recovery and service console"
```

## Task 14: Full Verification, HIL Matrix, and Release Gate

**Files:**
- Create: `docs/validation/hil-matrix.md`, `docs/validation/soak-test-log.md`
- Modify: `README.md`, `docs/protocol/i2c-control-protocol.md`

**Interfaces:**
- Consumes the complete firmware.
- Produces reproducible build evidence, device matrix, measured buffer/latency settings and remaining hardware risks.

- [ ] **Step 1: Run clean build and static checks**

```bash
bash scripts/preflight.sh
idf.py fullclean
bash scripts/build.sh
git diff --check
```

Record firmware size, component sizes, free/min heap at idle and streaming, and all warnings. Resolve correctness warnings before continuing.

- [ ] **Step 2: Execute the interoperability matrix**

Sink peers: Android, iOS and Windows. Source peers: at least two unrelated speaker/headphone vendors. For each, record discovery, pairing, reconnect, 32/44,1/48 kHz support, AVRCP commands, absolute volume, text metadata and Cover Art result/MIME/size.

- [ ] **Step 3: Tune and freeze Cover Art transport**

Measure 256- and 512-byte chunks at 400 kHz during audio. Select the size with zero audio underrun and best throughput, record pool high-water, and update both Kconfig default and protocol document. The v1 framing remains unchanged.

- [ ] **Step 4: Run the eight-hour audio tests**

For Sink, use an external I2S-master with known ppm offset and log FIFO level/drift ratio. For Source, capture the ESP32-generated BCLK/WS with a logic analyzer. Require zero panic/watchdog/underrun/overrun in nominal wiring and no audible Cover Art interference.

- [ ] **Step 5: Run 1000 mode switches and fault injection**

Inject Bluetooth disconnect, missing clocks, I2C polling pause, malformed frames, full event queue and Cover Art cancellation. Compare heap and task watermarks at cycles 1, 10, 100 and 1000; unexplained monotonic loss fails the gate.

- [ ] **Step 6: Final documentation and release build**

README must contain toolchain setup, pin Kconfig workflow, build/flash/monitor commands, I2S wiring roles, I2C polling contract, safe factory reset and known peer limitations. Produce a release build from a clean tree and record its SHA-256.

- [ ] **Step 7: Commit**

```bash
git add README.md docs/validation docs/protocol sdkconfig.defaults
git commit -m "test: validate Bluetooth audio module end to end"
```

## Final Completion Checklist

- [ ] `scripts/preflight.sh` proves ESP-IDF v6.0.2.
- [ ] Host and on-target unit tests pass.
- [ ] Clean firmware build and size checks pass.
- [ ] Both A2DP roles work at 32/44,1/48 kHz with the approved I2S clock roles.
- [ ] AVRCP controls, metadata and Cover Art work or return explicit peer capability errors.
- [ ] I2C protocol v1 is documented and tested at 400 kHz without IRQ.
- [ ] Eight-hour streams and 1000 mode switches meet acceptance criteria.
- [ ] Hardware-dependent GPIO values and measured Cover Art chunk size are recorded for the prototype board.
- [ ] Unverified peer-specific behavior is listed explicitly rather than represented as complete.
