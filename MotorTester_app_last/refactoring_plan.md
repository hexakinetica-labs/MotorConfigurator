# MKS Refactoring Plan (mks_can)

## Goal

Полностью перевести реализацию MKS в `drivers/interfaces/mks_can` и больше не использовать `mks_legacy` как рабочую ветку.

Ключевые требования:

- максимально простая архитектура без оверинжиниринга;
- неблокирующие методы и минимизация mutex;
- отдельный экземпляр оси на каждый привод;
- отдельный цикл оси с управляемой частотой;
- базовый класс + полиморфизм для 3 типов движения:
  - скорость;
  - абсолютная позиция;
  - стриминг;
- service-команды через потокобезопасные atomic-методы:
  - enable/disable;
  - home;
  - set zero;
  - clear errors;
  - estop;
  - parameter actions;
- приоритет service-команд над motion queue;
- если в текущем тике есть service-команда, motion queue в этом тике не трогаем;
- единственный путь движения мотора: отправка точки в очередь движения;
- телеметрия тоже через bounded очередь;
- если UI не успевает читать телеметрию, старые элементы удаляются (drop-oldest);
- EtherCAT не рефакторим.

## Target Architecture (minimal)

- `MksAxisAdapter` — тонкий фасад `motion_core::IAxis`.
- `MksAxisWorker` — осевой цикл, atomic service requests, motion/telemetry queues.
- `MksMotionModeBase` + 3 реализации:
  - `MksAbsolutePositionMotionMode`
  - `MksVelocityMotionMode`
  - `MksStreamingMotionMode`
- `MksCanBusManager` — единый владелец CAN I/O.

## Data Flow

1. UI/Runtime кладёт motion points в command queue оси.
2. Service actions выставляются через atomic request flags.
3. Осевая петля:
   - сначала обрабатывает service requests;
   - если service был обработан — завершает тик;
   - иначе берёт point из motion queue и через активный motion mode формирует CAN-команду.
4. CAN-команды отправляются через `MksCanBusManager`.
5. Телеметрия обновляется и складывается в bounded telemetry queue (drop-oldest policy).

## Integration

- Внешний `motion_core::IAxis` контракт сохраняется.
- Интеграция переводится на `drivers/interfaces/mks_can/*`.
- `CMakeLists.txt` и include-пути переводятся на `mks_can`.
- `drivers/interfaces/init_all_drivers.cpp` регистрирует runtime/scanner через `mks_can`.

## Constraints

- Изменения только в MKS-части.
- EtherCAT не затрагиваем функционально.
