# DroneCAN для К1946ВК035

Порт CAN0 и libcanard для К1946ВК035. Работает без ОС: приём через прерывание, обработка и передача — из основного цикла. Автор — **Regulo**. Плату для проверки разработал и предоставил **TDMLab**: [K1946VK035_BLDC_BOARD](https://github.com/TDMLab/K1946VK035_BLDC_BOARD).

## Что внутри

- `src/CAN_Driver.c` — CAN0, PB14/RX и PB15/TX, классический CAN с 29-битными идентификаторами. Скорости: 125, 250, 500 и 1000 кбит/с.
- `src/DroneCAN.c` — транспорт между драйвером и libcanard: очередь, ограниченное число кадров за один проход, очистка старых передач и сброс зависшего TX.
- `include/` — интерфейсы драйвера и транспорта.
- `third_party/libcanard/` — библиотека и стандартные сгенерированные DSDL из [VESC](https://github.com/vedderb/bldc).
- `captures/` — сеанс Saleae и экспорт CSV с реальной шины.

Порт не включает FOC, параметры двигателя и обработчики ESC. Прикладной проект задаёт ID узла и callbacks libcanard: какие передачи принимать и как их обрабатывать. В основной FOC-прошивке это делает `K1921VK035_POWER_BOARD/src/FOC_DroneCAN.c`; описание сервиса конфигуратора находится в `dsdl/regulo/foc/201.Exchange.uavcan` основной прошивки.

## Подключение

В новом проекте:

```sh
git submodule add https://github.com/RRegulo/DRONECAN_VK035.git third_party/DRONECAN_VK035
git submodule update --init --recursive
```

В сборку добавляются `src/CAN_Driver.c`, `src/DroneCAN.c`, `third_party/libcanard/canard.c` и нужные `.c` из `third_party/libcanard/dsdl`. В пути заголовков — `include`, `third_party/libcanard` и `third_party/libcanard/dsdl`. CMSIS и определения регистров предоставляет [SDK НИИЭТ](https://gitflic.ru/project/niiet/k1921vkx_sdk).

## Стенд и проверка

Плата TDMLab с TJA1042, MATEK H743-WING V3, ArduPlane 4.5.7, Mission Planner, 1 Мбит/с. CAN_H, CAN_L и GND соединены; два терминатора по 120 Ом дают около 60 Ом между H и L при выключенном питании. На стенде ID регулятора — 10, полётника — 20.

## Лицензия и источники

[GNU GPLv3](LICENSE). Уведомления об авторстве и MIT-лицензия libcanard сохранены в его исходниках. Порт транспорта использует libcanard из VESC; правила обмена задают [спецификация DroneCAN](https://dronecan.github.io/Specification/) и стандартные DSDL.

Спасибо **TDMLab** за плату.
