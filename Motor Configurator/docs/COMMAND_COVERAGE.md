# MKS CAN Command Coverage Matrix (DOC-FIRST)

## Принцип

**Единственный источник правды для команд — документация**:

1. `docs/mks_protocol_description.md`
2. `docs/MKS SERVO42&57D_CAN User Manual V1.0.6.pdf`

Код (`MksProtocol`, `MksAxisAdapter`, UI-дерево) обязан соответствовать документации, а не наоборот.

---

## Канонический реестр команд (по документации)

| Код | Каноническое название (из документации) |
|---|---|
| 30h | Read Encoder (Carry) |
| 31h | Read Encoder (Addition) |
| 32h | Read Motor Speed |
| 33h | Read Received Pulses |
| 34h | Read IO Status |
| 39h | Read Shaft Angle Error |
| 3Ah | Read EN Pin Status |
| 3Bh | Read Go Home Status |
| 3Dh | Release Stall Protection |
| 3Eh | Read Protection State |
| 3Fh | Restore Default Parameters |
| 41h | Restart Motor |
| 80h | Calibrate Encoder |
| 82h | Set Work Mode |
| 83h | Set Working Current |
| 84h | Set Subdivision |
| 85h | Set EN Pin Active Level |
| 86h | Set Motor Direction |
| 87h | Set Auto Turn-Off Screen |
| 88h | Set Locked Rotor Protection |
| 89h | Set Subdivision Interpolation |
| 8Ah | Set CAN Bitrate |
| 8Bh | Set CAN ID |
| 8Ch | Set Slave Respond/Active |
| 8Dh | Set Group ID |
| 8Fh | Set Key Lock |
| 90h | Set Home Parameters |
| 91h | Go Home |
| 92h | Set Current Axis to Zero |
| 9Ah | Set Mode0 |
| 9Bh | Set Holding Current |
| 9Eh | Set Limit Port Remap |
| F1h | Query Motor Status |
| F3h | Enable Motor |
| F4h | Run Position Relative Axis |
| F5h | Run Position Absolute Axis |
| F6h | Run Speed Mode |
| F7h | Emergency Stop |
| FDh | Run Position Relative Pulses |
| FEh | Run Position Absolute Pulses |
| FFh | Save/Clear Speed Mode |

---

## Покрытие в текущей реализации (вторично, для контроля)

| Код | Статус реализации | Комментарий |
|---|---|---|
| 31h, 32h, 3Eh, F1h | Используется | runtime telemetry polling |
| 82h, 83h, 84h, 85h, 86h, 87h, 88h, 89h, 8Dh, 8Fh, 9Bh, 9Eh | Используется | read/write через parameter API |
| 8Ah | Ограничено политикой | bootstrap/read-only в обычном patch |
| 8Bh | Используется с ограничениями | writable runtime-команда (смена CAN ID + remap), но non-persistable в AxisConfig |
| 8Ch | Запрещено политикой | запись блокируется в patch API |
| 91h, 92h, F3h, F5h, F6h, F7h, 3Dh | Используется | команды управления движением |
| 30h, 33h, 34h, 39h, 3Ah, 3Bh, 3Fh, 41h, 80h, 90h, 9Ah, F4h, FDh, FEh, FFh | Пока не задействовано runtime-path | есть в enum протокола, но не включено в активный flow |

---

## Требование к изменениям

Любое изменение в командах/именах/семантике делается в порядке:

1) сверка с документацией,
2) обновление документации проекта,
3) только потом изменение кода.

Если код противоречит документации — это баг кода.
