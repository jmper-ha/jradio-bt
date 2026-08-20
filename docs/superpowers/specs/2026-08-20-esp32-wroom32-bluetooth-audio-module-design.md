# ESP32-WROOM-32 Bluetooth Audio Module Design

**Дата:** 2026-08-20
**Статус:** утверждена пользователем
**Целевая платформа:** ESP32-WROOM-32, ESP-IDF v6.0.2

## 1. Назначение

Прошивка превращает ESP32-WROOM-32 в управляемый Bluetooth Classic аудиомодуль с двумя взаимоисключающими режимами:

- **Receiver / A2DP Sink:** модуль принимает SBC-аудио по Bluetooth, декодирует его в PCM и выдаёт через I2S.
- **Transmitter / A2DP Source:** модуль принимает PCM через I2S, кодирует его в SBC и передаёт Bluetooth-приёмнику.

Внешний контроллер управляет модулем как I2C-master. Отдельной линии IRQ нет: контроллер опрашивает состояние и очередь событий. UART0 остаётся свободным для прошивки, логов и сервисной консоли.

Первая версия включает A2DP, GAP, AVRCP, pairing, поиск устройств, автоматическое переподключение, текстовые метаданные и получение Cover Art. Wi-Fi, BLE, AAC и обновление firmware в изделии в первую версию не входят.

## 2. Аппаратные и аудиоограничения

- Целевой чип — исходный ESP32 в модуле ESP32-WROOM-32 с Bluetooth Classic и 4 МБ flash.
- GPIO I2C и I2S не фиксируются в исходном коде: они задаются через Kconfig и воспроизводимый `sdkconfig.defaults` для конкретной ревизии платы.
- На SDA и SCL устанавливаются внешние pull-up резисторы; внутренние pull-up ESP32 не считаются достаточными для рабочей шины 400 кГц.
- Аудиоформат фиксирован: Philips I2S, stereo PCM, signed 16-bit, MSB-aligned в 32-битном слоте каждого канала.
- Поддерживаемые Bluetooth/SBC частоты: 32, 44,1 и 48 кГц. Частота 22,05 кГц не используется при активном Bluetooth-аудио.
- MCLK для ESP32 не требуется. Используются BCLK, WS и одна линия данных для каждого направления.

Роли I2S зависят от режима:

| Режим | Направление PCM относительно ESP32 | I2S master | I2S slave |
|---|---|---|---|
| A2DP Sink | ESP32 выдаёт PCM по DOUT | внешний контроллер | ESP32 |
| A2DP Source | ESP32 принимает PCM по DIN | ESP32 | внешний контроллер |

В режиме Source ESP32 формирует BCLK/WS от APLL на согласованной A2DP-частоте. В режиме Sink внешний контроллер формирует BCLK/WS после получения требуемого формата от ESP32.

## 3. Выбранная архитектура

Используется один firmware-образ на ESP-IDF v6.0.2 и штатный стек Bluedroid. ESP-ADF не используется: прямой A2DP↔I2S тракт не требует его audio pipeline, а дополнительная framework-зависимость увеличила бы объём проверки совместимости.

Обе роли реализованы в одной кодовой базе, но одновременно активен только один набор A2DP/AVRCP-ролей. При переключении текущие профили корректно завершаются и деинициализируются, после чего запускаются профили противоположного режима. Полный restart ESP32 используется только как последний уровень восстановления после неудачной переинициализации Bluetooth.

Компоненты проекта:

- `mode_manager` — единственный владелец режима и главного конечного автомата;
- `bt_gap` — discovery, pairing, bonding и reconnect;
- `a2dp_sink` — приём SBC и выдача декодированного PCM;
- `a2dp_source` — получение PCM и передача SBC;
- `avrcp_bridge` — AVRCP Controller/Target, команды, состояние и metadata;
- `cover_art` — AVRCP 1.6 Cover Art Client и поток BIP/OBEX;
- `audio_i2s` — I2S channel lifecycle, DMA и смена master/slave/direction;
- `audio_buffer` — статически выделенные кольцевые PCM-буферы;
- `audio_drift` — компенсация рассогласования clocks в режиме Sink;
- `control_i2c` — I2C-slave mailbox, framing, responses и event queue;
- `settings` — NVS-настройки и сведения о последних устройствах;
- `diagnostics` — UART-консоль, логи и счётчики;
- `app_main` — детерминированная последовательность инициализации.

Bluetooth, I2S и I2C callbacks не выполняют длительную работу. Они копируют данные в заранее выделенные буферы и передают события задачам-владельцам. Выделение и освобождение heap запрещено в callbacks, DMA path и codec processing. Единственное исключение — `esp_a2d_audio_buff_alloc()` в отдельной Source sender task: при успешном `esp_a2d_source_audio_data_send()` Bluedroid забирает ownership буфера, поэтому его нельзя заменить повторно используемым application pool. Allocation failures считаются stream errors и измеряются отдельным счётчиком.

## 4. Аудиопотоки

### 4.1 A2DP Sink

```text
Bluetooth A2DP → SBC decoder → PCM FIFO → drift compensator
               → I2S TX DMA → внешний контроллер
```

После A2DP codec negotiation модуль публикует `AUDIO_FORMAT_CHANGED`. Внешний контроллер устанавливает BCLK/WS и отправляет `AUDIO_CLOCK_READY`. До подтверждения I2S-выход остаётся в mute, а поток не переводится в состояние `STREAMING`.

Bluetooth и внешний I2S-master имеют независимые генераторы. Поэтому `audio_drift` удерживает PCM FIFO около целевого заполнения, плавно изменяя отношение преобразования около 1,0. Используется медленный PI-регулятор и ограниченная ppm-коррекция. Первая реализация использует линейную интерполяцию; её спектральные характеристики и предел коррекции проверяются измерениями до фиксации production-настроек.

При underrun выход плавно переводится к нулю. При overrun применяется контролируемая коррекция с коротким crossfade. Оба события считаются ошибками качества и увеличивают диагностические счётчики.

### 4.2 A2DP Source

```text
внешний контроллер → I2S RX DMA → PCM FIFO → SBC encoder
                   → Bluetooth A2DP
```

ESP32 выбирает согласованную с Bluetooth-приёмником частоту, формирует BCLK/WS и сообщает формат внешнему контроллеру. Внешний контроллер работает как I2S-slave. Отдельная компенсация дрейфа не требуется, поскольку I2S и потребление PCM кодировщиком синхронизируются генератором ESP32.

### 4.3 Codec policy

В первой версии используется только обязательный A2DP codec SBC. Включается application-managed external codec path, рекомендованный ESP-IDF для новых разработок. AAC и другие необязательные codecs исключены.

## 5. Bluetooth, pairing и reconnect

`bt_gap` поддерживает:

- discoverable/connectable режим для A2DP Sink;
- discovery Bluetooth-приёмников для A2DP Source;
- передачу найденных имени, адреса, Class of Device и RSSI внешнему мастеру;
- подключение по выбранному Bluetooth-адресу;
- автоматическое подключение к последнему устройству отдельно для Sink и Source;
- просмотр и удаление bonding-записей;
- ограниченное по времени окно нового pairing.

По умолчанию применяется Secure Simple Pairing `Just Works`. Если внешний контроллер предоставляет пользовательский интерфейс, запрос PIN/passkey или numeric comparison передаётся событием I2C, а решение возвращается командой. Вне явно открытого pairing-окна модуль принимает только сохранённые устройства.

NVS хранит последний подтверждённый режим, имя модуля, настройки автоподключения и последние адреса отдельно для Sink и Source. Link keys управляются штатным Bluetooth/NVS механизмом ESP-IDF.

## 6. AVRCP и Cover Art

AVRCP обязателен в обоих режимах. Поддерживаются роли Controller и Target, необходимые для:

- play, pause, stop, next и previous;
- absolute volume и уведомлений о её изменении;
- playback state, position и duration;
- title, artist, album, track number, track count, genre и playing time;
- Cover Art image handle и получение изображения в режиме Sink.

Cover Art использует AVRCP 1.6 Cover Art Client и отдельный BIP/OBEX channel. Изображение не сохраняется целиком в RAM ESP32-WROOM-32. Поток организован так:

1. Модуль получает image handle и запрашивает свойства объекта.
2. Мастер получает MIME type, размеры и заявленный размер объекта.
3. Мастер выбирает original image, linked thumbnail либо вариант через BIP image descriptor.
4. Данные копируются из AVRCP callback в ограниченный пул блоков и читаются мастером по I2C.
5. Каждый блок содержит object ID, offset, length, final flag и CRC; мастер подтверждает его приём.
6. При медленном мастере применяется backpressure. Audio DMA и A2DP имеют более высокий приоритет.

Начальный размер I2C-блока находится в диапазоне 256–512 байт и окончательно выбирается после аппаратных измерений. Поля общего размера и смещения имеют ширину 32 бита, поэтому protocol framing не вводит искусственный предел размера изображения. Если peer не поддерживает Cover Art, модуль возвращает `NOT_SUPPORTED`, не нарушая аудио и текстовые metadata.

## 7. I2C protocol

ESP32 работает как 7-bit I2C-slave. Значения по умолчанию:

- address: `0x2A`;
- bus speed: до 400 кГц;
- polling interval мастера: 20–50 мс в активном режиме.

Адрес и GPIO конфигурируются через Kconfig. Протокол представляет собой версионированный mailbox, а не набор неатомарных mutable-регистров.

```text
magic | protocol_version | opcode | sequence | payload_length | payload | CRC16
```

Мастер записывает request frame и опрашивает короткий атомарный `STATUS`. Готовый response использует тот же `sequence`. Асинхронные события помещаются в подтверждаемую очередь с монотонными event IDs. Событие удаляется только после ACK мастера. Переполнение выставляет sticky-флаг и увеличивает loss counter.

Группы команд:

- protocol/device info и status snapshot;
- mode select и mode transition status;
- discovery, connect, disconnect и pairing response;
- bonded devices и local Bluetooth name;
- audio format, clock readiness и mute;
- AVRCP transport, volume, playback state и metadata;
- Cover Art properties, request, chunk read, ACK и cancel;
- diagnostics, reboot и factory reset.

Группы событий:

- boot ready, mode transition и recovery;
- discovery result/done;
- pairing request/result;
- A2DP/AVRCP connected/disconnected;
- audio format, clock request и stream state;
- metadata, playback, volume и Cover Art availability;
- I2S underrun/overrun, queue overflow и fatal error.

I2C ISR callbacks только перемещают фиксированные descriptors и используют ISR-safe FreeRTOS APIs. CRC, validation и выполнение command обрабатывает отдельная task. Все команды имеют определённый response или timeout; повтор request с тем же sequence не должен повторно выполнять неидемпотентное действие.

## 8. Конечный автомат и смена режима

Основные состояния:

```text
BOOT → IDLE → MODE_STARTING → DISCOVERABLE/SCANNING
     → CONNECTING → CONNECTED → CLOCK_WAIT → STREAMING
```

Служебные состояния: `DISCONNECTING`, `MODE_SWITCHING`, `RECOVERING` и `ERROR`.

Смена режима выполняется последовательно:

1. Запрет новых AVRCP и Cover Art операций.
2. Отмена текущего Cover Art transfer.
3. Mute, остановка I2S DMA и очистка PCM FIFO.
4. Остановка A2DP stream и Bluetooth disconnect с timeout.
5. Деинициализация текущих AVRCP и A2DP roles.
6. Перенастройка I2S direction и master/slave role.
7. Инициализация противоположных A2DP и AVRCP roles.
8. Переход в discoverable/scanning либо reconnect к последнему peer.
9. Публикация подтверждённого `MODE_CHANGED`.

Каждая асинхронная операция имеет generation ID. Callback от предыдущего поколения игнорируется. При timeout сначала полностью перезапускаются Bluetooth controller и Bluedroid. Если повторная инициализация неуспешна, целевой режим сохраняется и выполняется `esp_restart()`.

## 9. Ошибки и восстановление

- Все `esp_err_t` проверяются и логируются вместе с subsystem, operation, state и peer address при наличии.
- Повторные Bluetooth reconnect используют ограниченное число попыток и backoff; tight retry loop запрещён.
- Отсутствие I2S clocks удерживает состояние `CLOCK_WAIT`, не блокируя другие задачи.
- Потеря clocks во время stream останавливает DMA безопасно и публикует событие.
- Повреждённые I2C frames, неверные lengths и неизвестные opcodes не изменяют состояние.
- Входящие Bluetooth names, metadata и Cover Art считаются недоверенными; все lengths и offsets проверяются до копирования.
- Cover Art может быть отменён без остановки A2DP.
- Watchdog, heap watermark, queue high-water marks и причины последнего restart доступны через UART и I2C diagnostics.

## 10. Диагностика

UART0 использует ESP-IDF console/REPL с help и autocomplete. Минимальный набор команд:

- `status`, `version`, `mode`;
- `bt scan`, `bt connect`, `bt disconnect`, `bt bonds`;
- `audio status`, `audio buffers`, `audio stats`;
- `i2c status`, `i2c stats`;
- `heap`, `tasks`, `loglevel`;
- `reboot` и подтверждаемый `factory-reset`.

Application logs содержат стабильные tags, transitions, timeouts, retry counters и error codes. Шумные library logs понижаются точечно, без глобального отключения диагностической информации.

## 11. Конфигурация и flash

`sdkconfig.defaults` фиксирует target `esp32`, Bluedroid, Bluetooth Classic, A2DP, AVRCP 1.6 Cover Art, external codec path, новые I2S/I2C drivers и необходимые размеры стеков/буферов. Конкретные GPIO задаются board-specific Kconfig values.

Первая версия использует простой single-application layout для 4 МБ flash с NVS и factory application. OTA partitions и updater отсутствуют. Добавление обновления firmware является отдельным проектом, в котором заново выбираются transport, security policy и partition table.

Secure Boot v2 и flash encryption отключены в отладочных сборках. Перед серийным выпуском выполняется отдельный hardening этап с производственными ключами, отключением опасных debug paths и проверкой влияния защиты на сервисный процесс.

## 12. Проверка

### 12.1 Автоматические тесты

- CRC/framing, malformed frames и duplicate sequence I2C;
- event queue, ACK, overflow и atomic status snapshots;
- state transitions, timeouts, generation IDs и recovery;
- PCM FIFO, interpolation, PI controller и boundary conditions;
- metadata bounds и Cover Art chunk assembly;
- NVS defaults, persistence и factory reset.

### 12.2 Тесты на ESP32

- I2C-slave write/read/repeated-start и медленный polling;
- I2S DMA loopback для обеих ролей и всех частот;
- исчезновение и восстановление external BCLK/WS;
- многократная init/deinit A2DP и AVRCP;
- UART console и diagnostics snapshots.

### 12.3 Hardware-in-the-loop matrix

- Sink peers: Android, iOS и Windows;
- Source peers: несколько моделей колонок и наушников;
- sample rates: 32, 44,1 и 48 кГц;
- discovery, pairing, reconnect, AVRCP и absolute volume;
- metadata и Cover Art с разными players и MIME types;
- одновременное audio streaming и максимальная передача Cover Art.

### 12.4 Устойчивость и критерии приёмки

- непрерывный stream не менее 8 часов без panic, watchdog, underrun или overrun в штатной конфигурации;
- 1000 последовательных смен Sink↔Source без необъяснимой потери heap;
- восстановление после Bluetooth disconnect, исчезновения I2S clocks и временной остановки I2C polling;
- ни одна malformed I2C-команда не приводит к выходу за границы или изменению режима;
- аудио не прерывается и не деградирует из-за Cover Art transfer;
- потеря события всегда отражается sticky-флагом и loss counter;
- проект воспроизводимо собирается закреплённым ESP-IDF v6.0.2 без существенных warnings.

Экспериментальная часть Cover Art измеряет реальные MIME types, размеры, наличие thumbnails, скорость I2C, необходимый buffer pool и поведение peers при backpressure. По результатам фиксируются production-размер блока и рекомендуемый предел изображения без изменения версии основного framing protocol.

## 13. Не входит в первую версию

- firmware update/OTA и OTA partition layout;
- Wi-Fi и BLE;
- AAC и vendor codecs;
- микширование нескольких Bluetooth connections;
- одновременная активная работа A2DP Source и Sink;
- обработка частоты 22,05 кГц внутри ESP32;
- декодирование или масштабирование Cover Art на ESP32;
- production Secure Boot/flash encryption provisioning.

## 14. Официальная техническая основа

- ESP-IDF v6.0.2 Bluetooth Classic A2DP Source/Sink;
- AVRCP Controller/Target и AVRCP 1.6 Cover Art Client;
- ESP-IDF new channel-based I2S standard-mode driver;
- ESP-IDF new bus/device I2C slave driver;
- FreeRTOS queues/tasks и ESP-IDF NVS/console facilities.

Проект закрепляет точный tag ESP-IDF и не использует `master`/`latest` как build dependency.
