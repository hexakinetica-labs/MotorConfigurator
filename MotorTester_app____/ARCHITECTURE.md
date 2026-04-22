# ARCHITECTURE.md

## Last Updated: 2026-03-31 04:13:00

## Overview
Project for controlling and testing motors (MKS CAN, EtherCAT).

## UI Architecture
`AxisWorkspace` is the base class for axis-specific workspaces.
It uses `AxisWorkspaceControlPanel` and `AxisWorkspaceConfigPanel` to manage the UI components for the Control and Configuration tabs respectively.

### Workspace Hierarchy
- `AxisWorkspace` (Base)
    - `MksAxisWorkspace` (MKS CAN specific)
    - `EthercatAxisWorkspace` (EtherCAT specific)

### Panel Hierarchy
- `AxisWorkspaceControlPanel` (Base UI for control)
- `AxisWorkspaceConfigPanel` (Base UI for configuration)

## Changes
- **2026-03-31 04:13:00**: Simplified queued sine producer path for MKS + EtherCAT to direct runtime queue API (no UI intermediate queue, no JSON/HAL roundtrip).
  - Added direct queue stats API in `AxisManager`:
    - `queryMotionQueueStatsDirect(int axis_id) const`
    - continues to use existing `enqueueMotionBatchDirect(...)`.
  - Reworked `MksAxisWorkspace` queued sine path:
    - queue policy normalized to `capacity=50`, `prefill=40`, `low-watermark=20`, `refill-batch=20`.
    - producer now reads real axis queue size via `queryMotionQueueStatsDirect`.
    - generated sine points are pushed directly as `std::vector<motion_core::QueuedSetpoint>` via `enqueueMotionBatchDirect`.
    - removed UI-local trajectory deque buffering from the producer path.
  - Reworked `EthercatAxisWorkspace` with the same direct producer model and same queue constants (`50/40/20/20`).
  - Added `AxisWorkspace::transportProvidesTargetTrace()` hook and enabled it in both transport workspaces:
    - base workspace skips generic single-point `target` trace when transport-specific batch target trace is active,
    - removed double target plotting in queued-sine mode.
  - Result: producer/consumer path reduced to direct runtime queue operations with deterministic backpressure behavior and cleaner target graph source-of-truth.

- **2026-03-31 03:23:00**: Stabilization pass for shared sine streaming path (MKS + EtherCAT) and EtherCAT safety disable semantics.
  - Unified queue refill behavior in both `MksAxisWorkspace` and `EthercatAxisWorkspace`:
    - prefill only up to `kMotionQueuePrefillSamples`,
    - top-up only when below `kMotionQueueLowWatermarkSamples`,
    - bounded enqueue batches (`kMotionQueueTopUpBatchSamples`) instead of full-queue bursts,
    - explicit capacity clamping during refill.
  - Resulting behavior: removed bursty fill-to-capacity oscillation that produced random motor motion and growing `dropped` under sine mode.
  - Added EtherCAT-confirmed disable sequence in `HalHostService`:
    - for EtherCAT `DisableAxis` and `Stop`, host now performs repeated `Stop + set_enabled(false)` with telemetry verification,
    - disable completion is confirmed via statusword check (`Switch On Disabled` and not `Operation Enabled`),
    - returns timeout error if drive does not enter disabled state in bounded retries.
  - This makes E-Stop/disable path deterministic for EtherCAT instead of fire-and-forget command semantics.
- **2026-03-31 02:05:00**: Pre-production audit and fixes (6 bugs + 1 feature).
  - Bug 1: Default control source changed from HexaMotion to UI (Manual) — UI no longer blocked on startup.
  - Bug 2: IPC server (`HalHostService`) now auto-starts with runtime, enabling HexaMotion client connections on `127.0.0.1:30110`.
  - Bug 3: E-Stop is now global — stops ALL axes via `HalHostService` global Stop path, not just the clicked axis.
  - Bug 4: UI manual lease auto-acquired on runtime start — motion commands work immediately without manual combo toggle.
  - Bug 5: `publishHostState()` moved from `onFastTick` (250 Hz) to `slow_timer_` (1 Hz) to avoid widget thrashing.
  - Bug 6: Config panel mutating buttons (Apply, Save to Flash, Import) now disabled when HexaMotion owns or E-Stop is active.
  - Feature 7: Added HexaMotion Client tab in Network QToolBox showing IPC server status, endpoint, and connected client count; topology tree node appears/disappears based on HexaMotion client connections.
- **2026-03-31 01:44:00**: Integrated Mode Selector + global E-Stop semantics for MotorTester UI and HexaMotion HAL host client.
  - `HalHostService` now latches global E-Stop (`Stop`), publishes `estop_active` in state frames, and blocks non-safety operations while E-Stop is active; `ClearFault` clears the latch.
  - Added host-state propagation to UI via `AxisManager::hostStateUpdated` (owner/epochs/manual override/service mode/E-Stop) and surfaced owner + E-Stop in `MainWindow`.
  - Replaced manual takeover checkbox with explicit `Control Source` selector (`UI (Manual)` vs `HexaMotion`) and synchronized it with HAL ownership state.
  - `AxisWorkspace` now disables manual motion/config controls when control source is `HexaMotion` or when global E-Stop is active, while keeping E-Stop button always enabled.
  - `HalHostDriver` (HexaMotion side) now caches E-Stop state, rejects writes/service ops during E-Stop (except safety-allowed ops), drops pending stream frames on lease loss/E-Stop, and re-enters lease reacquire flow.
- **2026-03-30 21:26**: Unified runtime and fixed trajectory generation instability.
  - Combined MKS and EtherCAT `HalRuntime` instances into a single `unified_runtime_` in `AxisManager` to prevent thread priority competition and underruns.
  - Combined `HalHostService` instances into a single `host_service_` to resolve Manual Lease conflicts across transports.
  - Added reading and reporting of cycle metrics (`cmd_tx_hz`, etc.) in `AxisManager::onFastTick` so UI displays real performance.
  - Fixed `AxisWorkspace` to strictly use hardware `timestamp_ns` for graph X-axis, eliminating jitter and loops caused by OS timer drift.
  - Refactored sine wave trajectory generation in `MksAxisWorkspace` to be driven by actual driver queue size (filling when below watermark) rather than a fixed timer, ensuring perfectly smooth streaming.
- **2026-03-30 12:20**: Refactored `AxisWorkspace` to separate UI creation into `AxisWorkspaceControlPanel` and `AxisWorkspaceConfigPanel`. Created specialized panels for MKS and EtherCAT to fulfill the requirement of splitting the monolith.
- **2026-03-30 14:34**: Restored buildability of `motor_tester_gui` after partial HAL IPC migration in `AxisManager`.
  - Removed an accidental premature namespace close that caused `AxisManager` methods to be compiled as free functions.
  - Aligned control operations with current `hal_ipc::ControlOp` enum (`QueryMotionQueueStats`, `SetZero`, `Home`, `ListParameters`, `ReadParameters`, `SetPersistent`).
  - Switched local command dispatch from removed `ControlRequest/process_control_request` API to `HalControlFrameDto` + `HalHostService::execute_local_command`.
  - Kept manual takeover flow via `RequestManualLease/ReleaseManualLease` and `manualTakeoverChanged` signal.
  - Fixed telemetry field mismatch in fast UI tick (`actual_velocity_deg_per_sec` used for velocity output map).
- **2026-03-30 14:59**: Fixed runtime crash path in Qt workspace-to-manager dispatch.
  - Replaced invalid `QMetaObject::invokeMethod(..., "scheduleWatchAxis", ...)` with the existing slot `watchAxis(int,bool)`.
  - Replaced invalid `requestParameterList` invoke target with `requestListParameters`.
  - Disabled stale `saveConfigToFlash` invoke call in `AxisWorkspace` (no corresponding slot in current `AxisManager`), preventing deferred runtime method-resolution faults.
- **2026-03-30 15:04**: Fixed segfault when opening axis workspace from topology tree.
  - Removed cross-thread direct call `manager_->getAxisTransport(axis_id)` from UI thread (`MainWindow`).
  - Switched workspace selection logic to the already-known tree item transport tag (`mks_axis` / `ecat_axis`) set by topology population.
  - Updated `MainWindow::openAxisWorkspace` signature to accept axis type key, ensuring workspace creation no longer touches `AxisManager` synchronously across threads.
- **2026-03-30 15:08**: Fixed workspace-open crash after axis click caused by null UI handles in control panel.
  - Reimplemented `AxisWorkspaceControlPanel` widget construction and initialized all `Handles` pointers required by transport workspaces.
  - Added defensive null checks in `MksAxisWorkspace` and `EthercatAxisWorkspace` before dereferencing control panel handles (`speed_spin`, `accel_spin`, `target_pos_spin`, `target_slider`).
  - Result: workspace creation path no longer dereferences null UI pointers when opening MKS/EtherCAT axis tabs.
- **2026-03-30 16:18:35**: Restored legacy-matching axis workspace layout and recovered transport-specific runtime behavior after broken refactor.
  - Preserved the refactored split (`AxisWorkspace` base + transport-specific workspaces), but restored boss-required legacy control layout: colored action buttons on top, control panel on the left, telemetry panel on the right, and the scope monitor at the bottom.
  - Restored legacy UX detail in `AxisWorkspaceControlPanel`, including wheel-event protection for spin boxes and the original control hierarchy/visual emphasis.
  - Reimplemented missing runtime logic in `MksAxisWorkspace`: sine streaming, motion-queue configure/prefill/top-up, queue stat handling, target plotting, and queue cleanup before disable/home/set-zero service actions.
  - Reimplemented missing runtime logic in `EthercatAxisWorkspace`: mode-aware UI defaults, sine streaming for cyclic sync modes, queue prefill/top-up, queue cleanup, and live mode synchronization back into the UI.
  - Completed UI compilation integration by adding required Qt widget includes back into workspace translation units; `motor_tester_gui` now builds successfully again.
- **2026-03-30 17:33:00**: Refactored `AxisManager` to support independent MKS and EtherCAT runtimes, enabling mixed-session operation without destructive full-rebuilds.
  - Introduced two separate `HalRuntime` instances (`mks_runtime_`, `ethercat_runtime_`) and corresponding `HalHostService` instances.
  - Implemented a routing layer where `findAxis`, `listAxes`, and command dispatch functions now delegate to the appropriate transport-specific runtime based on axis ID.
  - Reworked `rebuildRuntimeFromCurrentConfig` to partition the master `HalRuntimeConfig` and build/rebuild MKS and EtherCAT runtimes independently.
  - Adapted `scanMotors` and `scanEthercatMotors` to merge discovered topology into the master configuration without affecting the other transport's runtime.
  - This change isolates the runtime lifecycles, allowing, for example, an EtherCAT hot-add without tearing down the active CANopen session, which was a critical architectural flaw.
