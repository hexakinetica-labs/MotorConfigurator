# MotorTester_app Architecture Notes

## 2026-04-05 04:26 (UTC+0400)

### EtherCAT Profile Position: aligned runtime path with XML semantics (target->controlword order + profile velocity object)

- В `EthercatAxisAdapter` для `Profile Position` сделаны правки в low-level path по ESI-логике:
  - подтверждён и зафиксирован порядок записи в RT-цикле: сначала целевые PDO (`0x607A`/`0x6060`), затем `0x6040`;
  - перенос отправки `controlword` в конец `process_cycle(...)`, чтобы pulse `bit4` не уходил раньше target.

- Для PP скорости добавлен явный путь через профильный объект:
  - при команде с `profile_speed_rpm` в режиме `ProfilePosition` выполняется SDO-запись в `0x6081:00` (`Profile velocity`),
    с конвертацией RPM -> instruction units/s через `gear_ratio` и `axial_resolution`.
  - Это отделяет профильную скорость PP от лимита `0x607F` и соответствует словарю/ESI.

- Существующий DS402 pulse для нового PP setpoint (`bit4` + `bit5`) сохранён,
  но теперь применяется в корректной фазе цикла относительно записи target.

## 2026-04-04 21:59 (UTC+0400)

### EtherCAT: added low-level Manual Homing (DI3) + fixed Profile Position set-point handshake

- Добавлен новый high-level режим `AxisMode::ManualHoming`:
  - `motion_core/types.h` — расширен enum режимов;
  - `EthercatAxisWorkspace` — добавлен пункт UI `Manual Homing (DI3)`;
  - `mks_axis_worker.cpp` — добавлены `case` для совместимости switch-веток после расширения enum.

- `Manual Homing` реализован **ниже AxisManager**, в `EthercatAxisAdapter` (RT path):
  - UI и `AxisManager` не содержат low-level логики DI/stop;
  - при `Home` в режиме `ManualHoming` адаптер выполняет движение в PV (`mode=3`),
    контролируя вход `DI3` из `0x60FD` (`bit2`);
  - скорость поиска фиксирована как безопасная константа `100 deg/s` по мотору;
  - при `DI3=1` движение останавливается и выставляется логический ноль через уже существующий
    механизм `zero_offset_counts_` + `HomeSwitchToZeroShiftDeg`.

- Исправлен `Profile Position` в `EthercatAxisAdapter`:
  - добавлен корректный DS402 new set-point handshake через `controlword bit4` (пульс);
  - добавлен `bit5` (`change immediately`) для PP-команд;
  - для каждого нового target в PP теперь формируется отдельный one-shot запрос set-point,
    что устраняет проблему игнорирования повторных position команд.

## 2026-04-04 19:10 (UTC+0400)

### EtherCAT: exposed input-port function parameters (0x2003:01..0x2003:08) in runtime parameter tree

- В словарь EtherCAT (`p100e_ethercat_dictionary.h`) добавлены параметры:
  - `0x2003:01` .. `0x2003:08` как `Input Port N Function`.
- Добавлены новые enum-id в `EthercatParameter`:
  - `InputPort1Function` .. `InputPort8Function`.
- Параметры сгруппированы в `Vendor Specific/Input Ports` и помечены как runtime-persistable.
- Формат данных: `INT16` (`SignedInteger`, `data_size=2`) согласно ESI (`DT2003`).

- За счёт существующего унифицированного flow без дополнительного helper:
  - `make_parameter_descriptors()` автоматически публикует новые параметры в UI parameter tree,
  - `read_parameters()` автоматически читает их через SDO,
  - `apply_parameter_patch()` автоматически применяет их через SDO write.

- Дополнительно подтверждено, что чтение состояния DI уже реализовано и сохранено:
  - `0x60FD` (`DigitalInputs`) читается в RT цикле и публикуется в telemetry/UI.

## 2026-04-04 18:39 (UTC+0400)

### EtherCAT: fixed Common/Homing parameter patch path (homing params from UI tree now apply)

- Исправлен критичный дефект в `EthercatAxisAdapter::apply_parameter_patch()`:
  - ранее для `ParameterDomain::Common` выполнялся ранний `continue`,
    из-за чего `Common/Homing` параметры из словаря EtherCAT не доходили до SDO-записи.

- После исправления:
  - локальные runtime-only common параметры (`HardwareGearRatio`, `HomeSwitchToZeroShiftDeg`)
    обрабатываются локально и завершаются `continue`;
  - остальные параметры доменов `Common` и `Ethercat` проходят общий словарный путь
    `find_ethercat_parameter_definition(...) -> convert_from_display_to_raw(...) -> request_sdo_write(...)`.

- Практический эффект:
  - параметры из `Common/Homing` (включая `HomingMethod`, `HomingSpeed*`, `HomeOffset`),
    изменённые из UI parameter tree, теперь применяются в привод, а не silently-skip.

## 2026-04-04 17:17 (UTC+0400)

### EtherCAT: home-switch-to-zero shift parameter + low-level apply on homing completion

- Добавлен новый общий параметр `CommonParameter::HomeSwitchToZeroShiftDeg`.

- В `EthercatAxisAdapter` параметр интегрирован как локальный persistable runtime parameter:
  - добавлен в `make_parameter_descriptors()` (group `Common/Homing`, unit `deg`),
  - добавлен в `read_parameters()` для экспорта в axis config,
  - добавлена обработка в `apply_parameter_patch()` с валидацией числового диапазона.

- Добавлено runtime-хранилище `home_switch_to_zero_shift_deg_` и low-level применение в RT цикле:
  - при `MaskHomingAttained` (если homing был активен) рассчитывается сдвиг в counts,
  - обновляется `zero_offset_counts_ = actual_counts + shift_counts`.

- За счёт уже существующей математики `zero_offset_counts_` это сразу влияет на:
  - actual telemetry position,
  - target command conversion в counts,
  без изменений UI/IPC API.

- Поскольку параметр помечен persistable и возвращается в `read_parameters()`,
  он автоматически участвует в export/import axis config через существующий `HalRuntime` flow.

## 2026-04-04 17:06 (UTC+0400)

### UI: DI отображение возвращено на bit0..bit7 + добавлен raw binary вид

- В `AxisWorkspace` для EtherCAT `digital_inputs` восстановлен прямой UI mapping:
  - `DI1 <- bit0`, `DI2 <- bit1`, …, `DI8 <- bit7`.

- В ту же строку `Digital Inputs` добавлено сырое бинарное представление младшего байта:
  - формат `raw[7:0]=01010101`.

- Итоговый вывод в UI теперь выглядит как:
  - `DI1=..., DI2=..., ... DI8=... | raw[7:0]=........`

- Изменение затрагивает только форматирование в UI (`axis_workspace.cpp`),
  backend путь чтения `digital_inputs` не менялся.

## 2026-04-04 16:41 (UTC+0400)

### UI: исправлен сдвиг нумерации DI для EtherCAT (физический DI1)

- По результату проверки на железе уточнено соответствие битов `0x60FD` к UI-меткам:
  физический `DI1` приходит как `bit1` (а не `bit0`).

- В `AxisWorkspace` обновлён formatter `Digital Inputs`:
  - было: `DI1 <- bit0 ... DI8 <- bit7`,
  - стало: `DI1 <- bit1 ... DI8 <- bit8`.

- Это правка только UI-интерпретации (label mapping).
  Backend чтение PDO `0x60FD` не менялось.

## 2026-04-04 16:26 (UTC+0400)

### UI: человекочитаемое отображение EtherCAT DI1..DI8 в Telemetry panel

- В `AxisWorkspaceControlPanel` (блок `Telemetry (Live)`) добавлена новая строка
  `Digital Inputs` с отдельным label (`lbl_digital_inputs`).

- В `AxisWorkspace::onTelemetryUpdated(...)` добавлено форматирование DI для EtherCAT:
  - источник: поле `digital_inputs` из telemetry map,
  - вывод: только первые 8 входов в явном виде
    `DI1=..., DI2=..., ... DI8=...`,
  - для не-EtherCAT транспорта отображается `N/A`.

- Реализация выполнена без новых API и без новых polling/timer потоков:
  используется существующий telemetry update pipeline.

## 2026-04-04 15:57 (UTC+0400)

### EtherCAT: realtime чтение DI (0x60FD) в существующем telemetry pipeline

- Реализовано минимальное расширение без новых public API-методов:
  - в `motion_core::AxisTelemetry` добавлено поле `digital_inputs` (`std::uint32_t`);
  - в `EthercatAxisAdapter::AtomicTelemetry` добавлен atomic-кэш `digital_inputs`.

- `EthercatAxisAdapter` переведён на realtime чтение DI через PDO:
  - в PDO map добавлен объект `0x60FD:00` (`ObjDigitalInputs`),
  - входной PDO `0x1A00` расширен до 5 entries,
  - зарегистрирован `off_digital_inputs_` в `ecrt_domain_reg_pdo_entry_list(...)`,
  - в `process_cycle(...)` читается `EC_READ_U32(domain_pd + off_digital_inputs_)` и публикуется в telemetry atomics.

- Значение DI проброшено вверх по уже существующему каналу телеметрии:
  - `EthercatAxisAdapter::read_telemetry()` заполняет `AxisTelemetry::digital_inputs`,
  - `AxisManager::onFastTick()` публикует `digital_inputs` в `telemetryUpdated(...)` map для UI.

## 2026-04-03 21:07 (UTC+0400)

### HAL/HexaMotion integration: удаление лишнего compatibility-wrapper

- Из `AxisManager` удалён `requestManualTakeover(bool)`.
- В public API оставлен только явный метод `setControlSource(...)` для переключения источника управления.
- Это убирает legacy-двусмысленность и фиксирует единый контракт ownership-switch без дублирующих entry points.

## 2026-04-03 20:48 (UTC+0400)

### HAL/HexaMotion integration: production safety cleanup (E-STOP lifecycle + legacy tail removal)

- `AxisManager` получил явный lifecycle для глобального E-STOP:
  - добавлен отдельный operator-path `resetEmergencyStop()`;
  - `estop_active_` больше не может быть снят неявно через per-axis flow.

- В `AxisManager::executeAxisOperation(...)` добавлен централизованный global E-STOP policy-gate:
  - при активном глобальном E-STOP блокируются операции, не относящиеся к safe recovery;
  - разрешены только recovery/monitor-safe операции (`Stop`, `ClearFault`, `ClearMotionQueue`, `QueryMotionQueueStats`, `None`).

- `requestManualTakeover(bool)` оставлен только как compatibility wrapper,
  authoritative API переключения ownership по-прежнему `setControlSource(...)`.

- Из `HalHostService` удалён неиспользуемый compatibility-хвост
  `execute_local_command(...)`.

Итог:

- `AxisManager` усилен как единственный владелец глобального control/safety state.
- `HalHostService` остался тонким IPC adapter без лишних API-хвостов.

## 2026-04-03 18:07 (UTC+0400)

### HAL/HexaMotion integration: production cleanup pass (AxisManager-first contract hardening)

- `HalHostService` дополнительно упрощен до thin IPC adapter:
  - удалены legacy helper-методы/кодеки и команды, не принадлежащие слою сервиса (`build_axis_command`, local parameter json helpers);
  - удален runtime-side fallback для global stop внутри сервиса;
  - для `Stop` с `service_axis_id <= 0` сервис теперь строго делегирует в `AxisManager` (`axis_id=0`), сохраняя единственного владельца business/service logic.

- `AxisManager` исправлен по критичным runtime/ownership хвостам:
  - `stopRuntime()` теперь останавливает `HalHostService` (исключено состояние «runtime stopped, IPC still listening»);
  - `stopRuntime()` публикует актуальный host state сразу после остановки.

- Зафиксирована корректная семантика глобального E-STOP:
  - убран implicit reset глобального `estop_active_` из `clearErrors(axis_id)`;
  - per-axis clear faults больше не снимает глобальный E-STOP latch.

- UI-path команды получили явную диагностику ошибок вместо silent-ignore:
  - `configureMotionQueue`, `enqueueMotionBatch`, `clearMotionQueue` теперь логируют причины отказов через `logMessage`.

## 2026-04-03 15:13 (UTC+0400)

### HAL/HexaMotion integration: keep-alive сохранён в send/recv контракте + строгий HexaMotion connectivity

- Поддержан инвариант обмена: без изменения формата, остаются те же две структуры
  `HalControlFrameDto` (send) и `HalStateFrameDto` (recv).
- Для keep-alive в HexaMotion HAL Host path добавлен no-op кадр в том же протоколе:
  - `HalHostDriver::sendKeepAlive()` отправляет `ControlOp::None` с `client_id = kClientIdHexaMotion`.
  - Это поддерживает соединение и telemetry refresh, не генерируя write-path motion ownership side-effects.
- Из `HalHostDriver::write(...)` удалён driver-side silent throttle (4 ms). Драйвер больше не дропает команды локально.

### Строгий признак подключения именно HexaMotion

- В `hal_ipc::HalIpcServer` добавлен отдельный счётчик `connected_hexamotion_clients_` и API
  `connected_hexamotion_client_count()`.
- Счётчик увеличивается только если от клиента действительно пришёл кадр с
  `client_id == kClientIdHexaMotion`, и уменьшается при disconnect.
- `HalHostService::HostStateSnapshot` расширен полем `connected_hexamotion_client_count`.
- `AxisManager` теперь использует именно этот признак для:
  - `RuntimeQueueIngressState::hexamotion_connected`,
  - публикации host state в UI (`hexamotion_connected_clients`).

### HardwareManager: keep-alive policy и unsupported service ops

- В Simulation mode keep-alive оставлен, но для `RealtimeInterfaceType::HalHost` переведён на
  `HalHostDriver::sendKeepAlive()` (no-op control frame), вместо генерации motion `write()` hold-пакетов.
- Для остальных backend'ов keep-alive-поведение `write(last_real_feedback_.joints)` сохранено.
- `HardwareManager::masterAxisAt(...)` для HAL Host на `InvalidArgument` теперь ведёт себя как
  compatibility-bypass (возвращает Success), синхронно с уже существующей логикой `setBrakeState(...)`.
  Это устраняет ложный fail-path для `zeroAxis()` в HAL Host режиме.

## 2026-04-03 04:56 (UTC+0400)

### HAL/HexaMotion integration: фиксация режима «HexaMotion = stream/read client only»

- `AxisManager::executeAxisOperation(...)` получил явный policy-gate для внешнего caller:
  - если `caller == HexaMotion`, разрешены только stream/read операции:
    - `StreamPoint`
    - `EnqueueMotionBatch`
    - `QueryMotionQueueStats`
    - `None`
  - любые service/business операции (enable/disable, stop, homing, mode/config, parameters, persistent, import/export) возвращают `PermissionDenied`.
- Это закрепляет целевой контракт: ownership бизнес-логики полностью у `AxisManager/UI`, а HexaMotion через HAL остается только клиентом точек/чтения.

### Поведение клиента HexaMotion при отказах policy

- В `HalHostDriver` добавлено корректное отображение `motion_core::ErrorCode::PermissionDenied` в `ErrorCode::InvalidArgument` (явный unsupported operation с точки зрения backend API).
- В `HalHostDriver::write(...)` исправлен возврат ошибки: теперь при неуспешном IPC exchange возвращается исходный `state_res.error()`, а не всегда `SocketReceiveFailed`.
  Это сохраняет диагностическую точность при policy/валидационных отказах.

### Проверка

- Выполнена сборка проекта (`cmake --build ...`) — успешно.
- Критичных ошибок компиляции не обнаружено; присутствуют только предупреждения (внешний `libusb` и legacy-unused helpers в `hal_host_service.cpp`).

## 2026-04-03 03:01 (UTC+0400)

### HAL/HexaMotion integration: единый источник host state через AxisManager

- `HalHostService` окончательно перестроен на модель thin IPC adapter без локального ownership-источника:
  - удалено поле `last_caller_` и зависимость публикации `motion_owner/service_owner` от «последнего IPC caller»;
  - добавлен `set_state_provider(...)`, через который сервис читает authoritative host/backend state извне.
- `AxisManager` подключает `HalHostService::set_state_provider(...)` и отдаёт единый snapshot,
  сформированный из своих authoritative полей `control_source` + `estop_active`.
- Публикуемый IPC state (`HalStateFrameDto`) теперь строится через `AxisManager`-state provider,
  поэтому UI path и IPC path видят одну и ту же owner-картину.
- Legacy lease-операции (`RequestManualLease` / `ReleaseManualLease`) сохранены только как compatibility no-op,
  без влияния на ownership/business decision logic.

Итог по слоям после зачистки:

- `AxisManager` = единый источник истины по control-source/owner-state и владелец business/service logic.
- `RuntimeQueueIngress` = маленький write-path arbiter (guard по `control_source`, `hexamotion_connected`, `estop_active`).
- `HalHostService` = тонкий IPC bridge: принимает кадры, делегирует операции в `AxisManager`,
  и публикует state из `AxisManager` provider.

## 2026-04-02 23:36 (UTC+04:00)

### HAL/HexaMotion integration: Финальная архитектурная зачистка слоев

Завершено приведение кода к строгому контракту «HexaMotion - клиент, AxisManager - хозяин»:

- `HalHostService`:
  - Полностью удалена внутренняя логика ownership (lease mechanics), estop и routing/filtering;
  - `estop_latched_` и `is_readonly_query` удалены;
  - Сервис теперь строго «тупой IPC-адаптер», который просто делегирует кадры в `AxisOperationHandler`.
- `AxisManager`:
  - Закреплен как единый authoritative API. Состояние управляется строго через `control_source` и `estop_active`;
  - `requestManualTakeover(bool)` оставлен только как thin wrapper поверх `setControlSource(...)`;
  - Исполнение команд через общий пайплайн `executeAxisOperation`, который теперь содержит всю business и service логику, возвращая результат.
- `HalHostDriver` (на стороне HexaMotion):
  - Удален локальный `estop_active_cache_`, который ошибочно блокировал `write()`;
  - Драйвер теперь работает строго как dumb stream/read client, транслирующий поток точек без собственных policy-решений;
  - Подтверждено, что сервисные операции (Zero/Brake/Master) возвращают `InvalidArgument` и игнорируются `HardwareManager` для данного backend'а.

## 2026-04-02 19:31 (UTC+04:00)

### HAL/HexaMotion integration: финализация целевого контракта AxisManager-first

- Зафиксирован целевой контракт маршрутизации:
  - `IPC -> HalHostService -> AxisManager -> RuntimeQueueIngress -> HalRuntime`
  - `UI  -> AxisManager    -> RuntimeQueueIngress -> HalRuntime`
- `AxisManager` закреплён как единый backend API и владелец service/business операций.
- `HalHostService` упрощён до тонкого IPC adapter:
  - lease/state-machine удалены,
  - команда от IPC декодируется и делегируется в единый axis-operation handler,
  - host-state отражает текущего IPC caller + runtime telemetry snapshot.
- `RuntimeQueueIngress` сохранён как маленький write-path gate только для потоковых точек/батчей.

### UI priority и HexaMotion как stream/read клиент

- Переключение источника координат (UI/HexaMotion) выполняется напрямую в `AxisManager`
  через `control_source_` и транслируется в `RuntimeQueueIngressState`.
- UI приоритетен и может возвращать управление без lease handshakes.
- При работе от HexaMotion UI телеметрия продолжает обновляться (observer/read path не блокируется).
- Со стороны HexaMotion `HalHostDriver` упрощён:
  - удалён lease-цикл (`RequestManualLease/ReleaseManualLease`, retry/epoch cache),
  - оставлены только `stream` (write) + `read` (feedback/state),
  - service-команды (zero/master/brake business semantics) не становятся источником бизнес-логики.

## 2026-04-02 12:24 (UTC+4)

### HAL/HexaMotion integration: single runtime-queue ingress + observer-safe UI

- Зафиксирована единая точка записи в runtime queue: `hal_host_service::RuntimeQueueIngress::submit_motion_batch(...)`.
- В `RuntimeQueueIngress` для write-path используется единый guard по состоянию:
  - `estop_active`
  - `control_source`
  - `hexamotion_connected` (для HexaMotion источника)

### Queue stats разделены на 2 сценария

- `query_motion_queue_stats(source, axis_id)` теперь тоже проходит через guard (тот же policy-контур, что и write).
- Добавлен observer path без ownership gate для мониторинга UI:
  - `query_motion_queue_stats_observed(axis_id)`

Это позволяет `AxisWorkspace` продолжать обновляться (включая queue stats), даже когда motion-owner = HexaMotion.

### Wiring changes

- `HalHostService::MotionQueueStatsIngress` упрощён до сигнатуры без caller:
  - было: `(OwnerRole caller, uint16_t axis_id)`
  - стало: `(uint16_t axis_id)`
- `HalHostService::apply_axis_op(QueryMotionQueueStats)` вызывает observer ingress.
- `AxisManager` пробрасывает stats через `RuntimeQueueIngress::query_motion_queue_stats_observed(...)`.
- `AxisManager::queryMotionQueueStatsDirect(...)` переведён на observer path (без блокировки по control source).

### Влияние на слои

- UI (`AxisWorkspace`) остаётся observer-only при внешнем владельце движения.
- Решение «кто может писать в runtime queue» централизовано внутри ingress, без дублирования в UI.

## 2026-04-02 17:47 (UTC+4)

### HAL/HexaMotion integration: Упрощение слоев

- `AxisManager` стал единственной точкой исполнения логики `AxisOperation`. Все локальные команды (перемещение, сброс, параметры) теперь не ходят через `HalHostService`, а сразу направляются в локальный `AxisManager::executeAxisOperation(...)`.
- `HalHostService` полностью освобождён от бизнес-логики обработки команд. Он выступает как тонкий IPC-адаптер. Методы маршрутизации `build_axis_command` удалены.
- `HalHostDriver` со стороны HexaMotion лишен права вызова сервисных операций (`setZero`, `setBrakeState`, `masterAxisAt`). Теперь он возвращает ошибку `InvalidArgument`, поскольку сервисными и калибровочными функциями теперь должен управлять исключительно `AxisManager` (со стороны MotorTester UI). HexaMotion имеет право только на `stream/read`.

## 2026-04-02 16:26 (UTC+4)

### HAL/HexaMotion integration: AxisManager как единый backend API

- `AxisManager` стал центральной точкой исполнения axis/service операций для IPC и UI:
  - добавлен `executeAxisOperation(...)` как общий dispatcher,
  - добавлен `executeStopAllAxes(...)` для глобального `Stop` (останов + очистка очередей по всем осям).

- `HalHostService` переведён в роль тонкого IPC adapter:
  - добавлен callback `set_axis_operation_handler(...)`,
  - `apply_axis_op(...)` теперь при наличии handler делегирует выполнение в `AxisManager`,
  - global-stop путь (`service_axis_id <= 0`) также делегируется в handler (если подключён).

- `RuntimeQueueIngress` сохранён как единый write-path gate:
  - потоковые команды (`StreamPoint`, `EnqueueMotionBatch`) в `AxisManager::executeAxisOperation(...)`
    всегда пишут в runtime queue только через `RuntimeQueueIngress::submit_motion_batch(...)`,
  - stats читаются через observer-path `query_motion_queue_stats_observed(...)`.

### Результат по слоям

- `AxisManager` = прозрачный единый backend API (владелец бизнес-логики осей/сервиса).
- `RuntimeQueueIngress` = маленький арбитр write-path в motion queue.
- `HalHostService` = тонкий IPC-адаптер без прикладной бизнес-логики (делегирование в `AxisManager`).

## 2026-04-02 22:12 (UTC+04:00)

### HAL/HexaMotion integration: Полная зачистка ownership-хвостов

- В `AxisManager` введен явный метод `setControlSource`, а `requestManualTakeover` переведен в разряд compatibility-wrapper.
- В `AxisManager` состояние упрощено: теперь authoritative-состоянием является только `control_source` и `estop_active`. Поля `motion_epoch`, `service_epoch`, `motion_owner`, `service_owner`, `manual_override_active` и `service_mode_active` переведены в derived/compatibility-значения для публикации. Это гарантирует отсутствие конфликтов между различными слоями при определении текущего состояния.
- Из `HalHostService` удален локальный стейт `estop_latched` и метод `is_readonly_query`, логика `estop` делегирована полностью в обработчик (`AxisManager`), что делает IPC-адаптер абсолютно тонким. Теперь `apply_control_frame` только осуществляет диспетчеризацию в handler без попыток fallback-логики. 
- В `HalHostDriver::setBrakeState` добавлено честное возвращение `ErrorCode::InvalidArgument` для unsupported операций через HAL Host. Это было подкреплено доработками в `HardwareManager`, чтобы он игнорировал `InvalidArgument` для `setBrakeState` и `masterAxisAt`, когда активен `HalHost` backend, исключая ложные сообщения об ошибках и восстанавливая честную картину разделения логики.
