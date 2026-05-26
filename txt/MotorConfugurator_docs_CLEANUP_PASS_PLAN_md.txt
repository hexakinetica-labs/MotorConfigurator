# Cleanup Pass Plan

Дата: 2026-03-21 16:06 UTC+4

## Цель

Сделать codebase более жёстким архитектурно:

- удалить legacy / deprecated entrypoints;
- убрать мёртвый и неиспользуемый код;
- сократить количество публичных методов;
- оставить только канонические control-path и parameter-path;
- снизить риск обхода архитектуры через случайные UI/API вызовы.

## Канонические границы

- `MainWindow / AxisWorkspace -> AxisManager`
- `AxisManager -> HalRuntime / IAxis`
- `HalRuntime -> IBusManager / IAxis`
- transport-specific детали не должны торчать выше `HalRuntime` и `IAxis`.

## Cleanup scope (текущий проход)

### 1. Удалить deprecated UI/control entrypoints


### 2. Удалить мёртвый код


### 3. Ужесточить visibility

- private по умолчанию;
- protected только если есть осознанное наследование;
- helper methods увести в private / cpp-local scope;
- убрать лишние `public slots`, если методы не являются реальным внешним entrypoint.

## Правило выполнения

Cleanup делается **не механически**, а contract-first:

1. сначала подтверждается, что method/field не нужен как архитектурный entrypoint;
2. затем он удаляется или скрывается;
3. после каждого этапа проект собирается.