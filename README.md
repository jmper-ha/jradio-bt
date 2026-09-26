# jradio-bt

*Read this in [English](README.en.md).*

Bluetooth-модуль для [jRadio](https://github.com/jmper-ha/jradio). У ESP32-S3,
на котором работает jRadio, нет классического Bluetooth — только BLE, а звук
(A2DP) ходит по классическому. Поэтому Bluetooth здесь — второй ESP32,
обычный WROOM-32 с этой прошивкой. Он сидит на той же шине I2S, что и ЦАП
jRadio, и слушается его по UART.

Умеет в обе стороны:

- **приём** — телефон подключается к устройству как к колонке: звук, название,
  исполнитель, альбом, обложка, позиция, громкость в обе стороны, пауза и
  перелистывание с ручки устройства;
- **передача** — всё, что играет устройство (радио, файлы, Яндекс, DLNA),
  уходит на Bluetooth-колонку или наушники: с её кнопок работают пауза,
  следующая/предыдущая станция, колесо громкости.

Что при этом видно на устройстве и в веб-интерфейсе и как этим пользоваться —
в [документации jRadio](https://github.com/jmper-ha/jradio/blob/main/doc/usage.md#bluetooth);
как подключить — в [описании железа](https://github.com/jmper-ha/jradio/blob/main/doc/hardware.md#bluetooth-модуль-jradio-bt).

## Провода

| Модуль (WROOM-32) | jRadio (ESP32-S3) | Что |
|---|---|---|
| GPIO 17 (TX) | GPIO 14 | UART 921600 8N1 |
| GPIO 16 (RX) | GPIO 13 | |
| GPIO 26 | GPIO 18 (BCLK) | общая шина I2S, через 33–47 Ом с каждой стороны |
| GPIO 25 | GPIO 17 (LRCK) | |
| GPIO 22 | GPIO 16 (DOUT) | тот же провод идёт и в DIN ЦАПа |
| GND | GND | |

Пины меняются в `idf.py menuconfig` → **jradio-bt**. Модуль никогда не берёт
шину сам: три ножки I2S — входы, пока хост не пришлёт `SET_MODE sink`, а хост
шлёт его, уже отпустив свои. В режиме передачи ведущий — S3, модуль только
слушает.

Антенна модуля рядом с антенной S3 мешает Wi-Fi ровно в те секунды, когда
модуль передаёт на всех каналах подряд: поиск колонок и дозвон до выключенной.
Поэтому поиск длится пять секунд и только по кнопке, а колонке звонят три раза
и дальше ждут, когда она позвонит сама. Обычная передача звука Wi-Fi не мешает —
адаптивные прыжки по частотам обходят его канал.

## Прошивка

**Из браузера, без ESP-IDF:** [jmper-ha.github.io/jradio-bt](https://jmper-ha.github.io/jradio-bt/) —
подключить модуль по USB, нажать «Прошить», выбрать порт. Chrome или Edge на
компьютере. Страница собирается автоматически из тега `v*` и из каждого push в `main`
(`.github/workflows/pages.yml`): прошивка компилируется в образе ESP-IDF 5.5.5,
три `.bin` и манифест публикуются на GitHub Pages; прошивает esptool-js прямо
в браузере.

**Из исходников:** одна сборка на все платы — экрана у модуля нет. Нужен ESP-IDF
5.5.x, тот же, что у jRadio; как его поставить — в
[инструкции jRadio](https://github.com/jmper-ha/jradio/blob/main/doc/toolchain.md).

В VS Code с расширением ESP-IDF: открыть папку, Terminal → Run Task… →
**ESP-IDF: Flash** (или «Build, Flash & Monitor»). Задачи зовут те же обёртки,
что и у jRadio — `tools/idf.sh` и `tools/idf.ps1` под Windows, — которые сами
находят установленный ESP-IDF и порт; если плат подключено две, порт модуля
задаётся переменной `ESPPORT`. Цель `esp32` уже задана в `sdkconfig.defaults`.

Из терминала:

```bash
source <esp-idf>/export.sh
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor    # порт USB-консоли модуля
```

Версия — в `main/version.h` и в теге git того же номера; точный коммит модуль
печатает при старте (`jradio-bt 1.0.0 (v1.0.0), protocol 1`), а хост показывает
номер на странице «Об устройстве».

## Как устроено

- `components/jbt_proto/` — протокол UART: SLIP-рамка, заголовок
  `type flags seq len`, CRC-16/CCITT, строки в TLV. Один файл на обе стороны:
  jRadio держит его копию и тесты проверяют, что она совпадает байт в байт.
  Все сообщения описаны в `jbt_proto.h`, больше нигде.
- `components/jbt_link/` — приёмная задача UART, отправка, зеркало лога
  (всё, что модуль пишет в свою консоль, хост видит как `bt_link: bt: …`).
- `components/bt_stack/` — контроллер и Bluedroid, имя устройства.
- `components/a2dp_sink/` — приём: A2DP, AVRCP-метаданные, обложка по BIP,
  громкость; `components/audio_out/` — ведущий I2S с кольцевым буфером.
- `components/a2dp_source/` — передача: поиск, подключение, AVRCP в обе
  стороны (громкость колонке, кнопки от неё); `components/audio_in/` —
  ведомый I2S с пересчётом любой частоты хоста в 44,1 кГц.
- `main/` — диспетчер сообщений и смена ролей (Bluedroid умеет одну за раз),
  память телефона и колонки в NVS.

Bluedroid и его ловушки, на которые ушло больше всего времени, описаны в
комментариях там, где они сработали — в `a2dp_source.c` (фильтр кнопок,
который отвергает весь набор из-за одного кода) и в `main.c` (имя до старта
стека, порт I2S при смене роли).

## Проверка без S3

USB-UART на пины 16/17 (не USB-консоль модуля), телефон под рукой, `pyserial`:

```bash
tools/jbt.py -p /dev/ttyUSB1 ping          # PONG protocol=1 firmware=1.0.0
tools/jbt.py -p /dev/ttyUSB1 mode sink     # MODE_ACK mode=sink result=ok
tools/jbt.py -p /dev/ttyUSB1 pairing on    # телефон видит модуль
tools/jbt.py -p /dev/ttyUSB1 monitor       # STATUS, TRACK, POSITION, VOLUME…
tools/jbt.py -p /dev/ttyUSB1 mode source
tools/jbt.py -p /dev/ttyUSB1 scan          # колонки вокруг
tools/jbt.py -p /dev/ttyUSB1 connect AA:BB:CC:DD:EE:FF
```

В режиме `sink` звук уже идёт на пины 25/26/22 — можно подцепить любой
I2S-ЦАП. `tools/jbt.py` — второй, независимый разбор протокола; он и прошивка
привязаны к одному тестовому вектору.

Тесты протокола — без ESP-IDF: `bash tests/run_host_tests.sh` (gcc с
санитайзерами).

## История

В ветке `feature/esp32-bt-module` лежит августовская попытка с управлением
по I2C — не слита и не нужна.
