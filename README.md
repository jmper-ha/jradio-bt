# jradio-bt

Bluetooth‑модуль для [jRadio](https://github.com/jmper-ha/jradio): второй ESP32
(WROOM‑32, классический — только у него есть Bluetooth Classic), который
принимает звук с телефона (A2DP sink) и отдаёт звук на наушники (A2DP source),
сообщает хосту метаданные трека и обложки (AVRCP) и слушается его по UART.
Звук ходит по той же шине I2S, что и ЦАП jRadio; кто её тактирует — зависит от
режима.

Состояние: **шаг 2 — приём.** Телефон подключается по A2DP, звук идёт в I2S
(модуль — ведущий шины в этом режиме), AVRCP отдаёт название/исполнителя/
альбом/длительность и позицию, громкость с телефона и с хоста сходятся
(absolute volume). По UART работают `PING`, `GET_STATUS`, `SET_NAME`,
`PAIRING`, `SET_MODE off|sink`, `CONNECT`, `DISCONNECT`, `PASSTHROUGH`,
`SET_VOLUME`, зеркало лога; `source` и обложки — дальше по плану.

## Провода

| Модуль (WROOM) | jRadio (S3) | Что |
|---|---|---|
| GPIO 17 (TX) | GPIO 14 | UART 921600 8N1 |
| GPIO 16 (RX) | GPIO 13 | |
| GPIO 26 | GPIO 18 (BCLK) | общая шина I2S, через 33–47 Ом с каждой стороны |
| GPIO 25 | GPIO 17 (LRCK) | |
| GPIO 22 | GPIO 16 (DOUT) | тот же провод — в DIN PCM5102 |
| GND | GND | |
| 3V3 | 3V3 | до 200 мА в пиках |

Пины и скорость — в `idf.py menuconfig` → **jradio-bt**. Модуль никогда не
трогает шину сам: три ножки I2S — входы, пока хост не пришлёт `SET_MODE sink`
(и хост шлёт его, уже отпустив свои).

## Проверка без S3

USB‑UART на пины 16/17, телефон под рукой:

```bash
tools/jbt.py -p /dev/ttyUSB1 ping          # PONG protocol=1 firmware=0.1
tools/jbt.py -p /dev/ttyUSB1 mode sink     # MODE_ACK mode=sink result=ok
tools/jbt.py -p /dev/ttyUSB1 pairing on    # ACK; телефон видит «jRadio»
tools/jbt.py -p /dev/ttyUSB1 monitor       # STATUS, TRACK, POSITION, VOLUME…
```

Звук при этом уже идёт на пины 25/26/22 — можно подцепить любой I2S‑ЦАП.

## Сборка

```bash
source <idf>/export.sh          # ESP-IDF 5.5.x
idf.py set-target esp32         # один раз
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
bash tests/run_host_tests.sh    # протокол: gcc + санитайзеры, без IDF
```

## Протокол

Описан в `components/jbt_proto/include/jbt_proto.h` — один файл на обе
стороны и на тесты. SLIP‑рамка, заголовок `type flags seq len`, CRC‑16/CCITT,
строки в TLV. Второй, независимый разбор — в `tools/jbt.py`, и оба привязаны к
одному тестовому вектору.

Поговорить с модулем с ПК, через USB‑UART на его пинах 16/17 (не через
консоль USB модуля):

```bash
tools/jbt.py -p /dev/ttyUSB1 ping
tools/jbt.py -p /dev/ttyUSB1 status
tools/jbt.py -p /dev/ttyUSB1 name "jRadio кухня"
tools/jbt.py -p /dev/ttyUSB1 pairing on
tools/jbt.py -p /dev/ttyUSB1 monitor      # все кадры, включая зеркало лога
```

Нужен `pyserial`.

## План

1. ~~Каркас, UART‑протокол, PING/STATUS/LOG, PC‑утилита~~
2. ~~Приём: A2DP sink → I2S master, absolute volume, метаданные; на S3 — источник «Bluetooth»~~ (не проверено на железе)
3. Обложки: BIP на модуле, `COVER_*`, кэш на S3
4. Передача: I2S slave RX → A2DP source, поиск и подключение наушников
5. Дрейф буфера, переключение шины без щелчков

В ветке `feature/esp32-bt-module` лежит августовская попытка с управлением по
I2C — не слита, но `pcm_ring`/`audio_drift` оттуда могут пригодиться на шаге 5.
