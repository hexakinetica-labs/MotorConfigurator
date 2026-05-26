# Safety Audit — MKS/MS Motor Control Path

Date: 2026-04-28 05:52 Asia/Tbilisi

## Scope

Audit focus:

- `ui/qt/mks/*`
- `ui/qt/mks/axis_manager/*`
- `hal_host_service/runtime_queue_ingress.h`
- `motion_core/src/hal_runtime.cpp`
- `drivers/interfaces/mks_can/*`
- selected `drivers/interfaces/ethercat/*` paths for comparison

Goal: find bugs that can potentially lead to uncontrolled behavior of MKS/MS motors, especially after the recent MKS control-panel refactor.

---

## Executive Summary

The audit found several defects that materially weaken motor-safety guarantees in the current architecture.

Most important problems:

1. **Service commands bypass the same safety/ownership gate that motion commands use.**
2. **Runtime shutdown/rebuild can lose `Disable` before the frame reaches hardware.**
3. **MKS absolute-position path has a speed-unit mismatch that can command much higher RPM than requested.**
4. **MKS multi-axis homing can mark an axis `Completed` before final `SetZero` is actually executed on the drive.**
5. **`enable_on_start` can energize axes before the startup safety baseline disables them again.**

These are not cosmetic issues. They are plausible paths to unexpected enable, motion, or stale-command execution.

---

## Severity Legend

- **CRITICAL** — can directly cause unexpected motion / unsafe enable / failed stop path.
- **HIGH** — strong safety degradation with realistic field impact.
- **MEDIUM** — can contribute to hazardous behavior or operator deception.

---

## Findings

### 1. Service-command path bypasses ownership and E-STOP safety gate

**Severity:** CRITICAL

**Evidence**

- `hal_host_service/include/hal_host_service/runtime_queue_ingress.h:54-67`
  - `enqueueServicePoint(...)` ignores `source` and does not check `estop_active`.
- `hal_host_service/include/hal_host_service/runtime_queue_ingress.h:71-99`
  - ownership / E-STOP authorization exists only inside `authorize_motion(...)`, i.e. only for motion commands.
- `ui/qt/mks/axis_manager/axis_manager_motion.cpp:98-195`
  - UI service commands go through `runtime_queue_ingress_->enqueueServicePoint(...)`.

**Why this is dangerous**

The system enforces hard gating for motion commands, but **not** for service commands such as:

- `Enable`
- `Disable`
- `Home`
- `SetZero`
- `SetOperatingMode`
- `ClearMotionQueue`

As a result, the following invariants are broken:

- ownership arbitration `UI vs HexaMotion` is incomplete,
- global `E-STOP` is not a true hard gate for service traffic,
- safety depends too much on UI button-disable policy instead of runtime enforcement.

**Practical failure scenario**

While HexaMotion owns motion, UI-side service requests can still reconfigure mode / home / enable. Under global E-STOP, a queued or programmatic service call can still pass through the ingress path.

**Recommended fix**

Make `enqueueServicePoint(...)` enforce the same state gate as motion commands, with a very small explicit allowlist for recovery-only operations.

---

### 2. Global E-STOP latch still allows `EnableAxis`

**Severity:** CRITICAL

**Evidence**

- `ui/qt/mks/axis_manager/axis_manager_hal_service.cpp:58-69`
  - when `estop_active_` is true, `EnableAxis` is treated as a `recovery_op` and is allowed.
- `hal_host_service/include/hal_host_service/runtime_queue_ingress.h:54-67`
  - service ingress does not independently block enable during E-STOP.

**Why this is dangerous**

A latched E-STOP should not allow re-enable of the power/motion path without an explicit, controlled reset flow. Allowing `EnableAxis` while E-STOP is still active defeats the semantics operators expect from an emergency stop.

**Practical failure scenario**

Operator or remote client hits global E-STOP, but another client still sends `EnableAxis`. The axis may be re-energized while the system still advertises `estop_active=true`.

**Recommended fix**

Do not allow `EnableAxis` while the global E-STOP latch is active. Recovery path should be:

1. explicit operator reset of E-STOP state,
2. only then allow enable/home/motion.

---

### 3. Runtime stop / rebuild can drop `Disable` before hardware sees it

**Severity:** CRITICAL

**Evidence**

- `motion_core/src/hal_runtime.cpp:282-291`
  - rollback on startup failure only enqueues `Disable`, then immediately stops buses.
- `motion_core/src/hal_runtime.cpp:356-368`
  - `HalRuntime::stop()` also only enqueues `Disable`, then immediately stops bus managers.
- `drivers/interfaces/mks_can/manager/mks_can_bus_manager.cpp:162-185`
  - stopping the MKS bus manager stops runtime loop and closes the CAN port.

**Why this is dangerous**

`Disable` is queued, not guaranteed delivered. If bus shutdown happens first, the frame may never leave the process. In that case the drive can remain enabled even though the runtime is considered stopped/closed/rebuilt.

**Practical failure scenario**

- runtime rebuild after scan,
- runtime close,
- startup failure rollback,
- operator stop runtime.

All of these paths can produce a false feeling that motors were safely disabled.

**Recommended fix**

Before stopping the bus:

- send an explicit stop/disable path that is synchronously drained or acknowledged,
- or add a bounded flush stage before bus shutdown,
- or use direct bus-manager emergency disable path during teardown.

---

### 4. MKS absolute-position command uses inconsistent speed units

**Severity:** CRITICAL

**Evidence**

- `ui/qt/mks/axis_workspace/axis_workspace_control_panel.cpp:106-110`
  - UI speed input is labeled in **RPM**.
- `ui/qt/mks/axis_manager/axis_manager_motion.cpp:68-74`
  - if no explicit target velocity is given, `profile_speed_rpm` is converted to `deg/s` via `rpm * 6` and stored in `MotionCommandPoint.velocity`.
- `drivers/interfaces/mks_can/motion/mks_absolute_position_motion.cpp:45-48`
  - MKS absolute motion builder interprets `point.velocity` as **RPM** and writes it directly into the drive speed field.
- `ui/qt/mks/mks_all_axes_control_workspace.cpp:437-441`
  - all-axes MKS panel sends `profile_speed_rpm = 300`.

**Why this is dangerous**

The same logical field is treated as:

- **deg/s** in `AxisManager`,
- **RPM** in MKS absolute-motion builder.

So a request for `300 rpm` becomes:

- converted to `1800 deg/s`,
- then treated as `1800 rpm` by MKS command generation.

This is a large command amplification and is exactly the kind of bug that can cause unexpectedly aggressive movement.

**Recommended fix**

Unify semantics immediately:

- either keep `MotionCommandPoint.velocity` in `deg/s` end-to-end and convert to motor RPM only inside the MKS builder,
- or carry explicit profile RPM separately.

Do not mix UI RPM and runtime deg/s in the same field.

---

### 5. Multi-axis MKS homing can advance before final `SetZero` really happened

**Severity:** HIGH

**Evidence**

- `drivers/interfaces/mks_can/adapter/mks_axis_adapter.cpp:342-352`
  - status becomes `Completed` immediately after `SetZero` is only **enqueued** internally.
- `ui/qt/mks/mks_all_axes_control_workspace.cpp:139-149`
  - all-axes sequence moves to the next axis as soon as telemetry status becomes `Completed`.

**Why this is dangerous**

`Completed` here means only: internal request accepted into software queue. It does **not** mean:

- CAN frame was transmitted,
- drive accepted it,
- zeroing was actually applied,
- post-zero telemetry was observed.

This allows the next axis to start while the previous axis may still be finishing the last critical homing action.

**Recommended fix**

Only publish `Completed` after a real completion condition, for example:

- final `SetZero` transmit confirmed,
- and/or position/status readback confirms zero reset,
- and/or a dedicated post-zero completion state is observed.

---

### 6. `enable_on_start` energizes axes before the startup safety baseline disables them

**Severity:** HIGH

**Evidence**

- `motion_core/src/hal_runtime.cpp:312-321`
  - `HalRuntime::start()` enqueues `Enable` for axes with `enable_on_start=true`.
- `ui/qt/mks/axis_manager/axis_manager_core.cpp:533-547`
  - only after `unified_runtime_.start()` returns does `startRuntimeHeadless()` apply baseline `Disable + SetZero`.

**Why this is dangerous**

There is a real window where the axis can become enabled before the “safe baseline” takes it back down. That violates the intention of a deterministic startup-safe state.

**Recommended fix**

Choose one policy and enforce it consistently:

- either remove `enable_on_start` for safety-critical MKS/MS startup,
- or apply baseline before any enable can be issued,
- or explicitly block startup enable when baseline policy is active.

---

### 7. Aborting MKS homing by `Disable` does not clear queued motion commands

**Severity:** HIGH

**Evidence**

- `ui/qt/mks/mks_all_axes_control_workspace.cpp:499-500`
  - abort path for sequence stop sends only `Disable` to the active axis.
- `drivers/interfaces/mks_can/adapter/mks_axis_adapter.cpp:119-138`
  - on `Disable`, adapter cancels homing state, but does not clear motion queue.
- `drivers/interfaces/mks_can/axis/mks_axis_worker.cpp:245-262`
  - `Disable` only emits `EnableMotor=0`; queue clear exists only under `ClearMotionQueue`.

**Why this is dangerous**

If a homing-related move-to-offset or another queued point already exists in the worker queue, disabling the axis does not prove that stale queued motion is gone. After later re-enable or mode re-entry, stale points may still be available for execution.

This is especially risky because the UI currently treats `Disable` as a “safe abort”.

**Recommended fix**

On MKS homing abort / disable during procedure:

- clear the motion queue explicitly,
- cancel homing state machine,
- then disable the axis.

---

### 8. Startup baseline is applied before per-axis config files are re-applied

**Severity:** MEDIUM

**Evidence**

- `ui/qt/mks/axis_manager/axis_manager_core.cpp:424-467`
  - runtime is started first; only afterward are per-axis config files applied.
- `ui/qt/mks/axis_manager/axis_manager_core.cpp:480-507`
  - baseline `Disable + SetZero` is issued before those config files are re-applied.

**Why this matters**

Behavior at startup can use stale runtime settings for at least one phase. This is less direct than the issues above, but still risky if future logic depends on configured sign/limits/offset/runtime identity during startup sequencing.

**Recommended fix**

Document and enforce a strict order:

1. open runtime,
2. apply required axis config,
3. apply baseline,
4. allow explicit operator enable.

---

## Additional Observations

1. `AxisWorkspaceControlPanel` generic default is `1800 RPM / 100%` (`ui/qt/mks/axis_workspace/axis_workspace_control_panel.cpp:106-116`).
   MKS-specific workspace later overrides this to safer defaults, but the generic baseline is still aggressive and easy to misuse if transport-specific initialization regresses.

2. Several MKS command-send results are intentionally ignored in the async path (`MksAxisWorker`, `MksCanBusManager`).
   For ordinary telemetry this is acceptable, but for `Disable`, `EmergencyStop`, `Home`, `SetZero` stronger observability is recommended.

---

## Recommended Fix Order

1. **CRITICAL:** hard-gate all service commands in `RuntimeQueueIngress`.
2. **CRITICAL:** forbid `EnableAxis` while global E-STOP latch is active.
3. **CRITICAL:** add guaranteed stop/disable flush before runtime/bus shutdown.
4. **CRITICAL:** fix MKS speed-unit contract end-to-end.
5. **HIGH:** make MKS homing `Completed` reflect real hardware completion.
6. **HIGH:** clear MKS motion queue on homing abort / disable path.
7. **HIGH:** remove or strictly redefine `enable_on_start` when baseline safety policy is active.

---

## Bottom Line

The project currently has multiple realistic paths where software can:

- re-enable or re-home motors under a supposed safety stop,
- stop runtime without guaranteed physical disable,
- over-command MKS speed due to unit mismatch,
- or move to the next homing step before the previous axis is actually finalized.

These findings should be treated as safety bugs, not just refactor debt.