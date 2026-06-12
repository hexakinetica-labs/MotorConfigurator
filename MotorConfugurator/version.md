# Software Version
1.2.5

## Description

- Fixed MKS CAN telemetry/configuration deadlock risk after loading or applying parameters by making synchronous parameter read/write operations drain their own bounded CAN responses while telemetry polling is paused.
- Fixed cross-thread telemetry delivery by marshalling `AxisTelemetryController` updates onto the Qt object thread before emitting UI-facing telemetry signals.
- Corrected telemetry UI semantics: MKS no longer displays fake digital inputs, and MKS CAN protocol event rates are labeled as events instead of control-loop frequency.
- Fixed MKS telemetry recovery after loading/applying axis parameters by pausing async telemetry polling during configuration and reasserting the MKS response policy after successful parameter patches.
- Fixed GUI freezes/stuck busy states in configuration and parameter-tree workflows by making parameter requests explicitly completion-driven, rejecting duplicate reads/lists with typed Busy diagnostics, delaying readback until write completion, and serializing axis config worker operations.
- Decomposed AxisManager into AxisLifecycleController, AxisMotionController, AxisConfigController, and AxisTelemetryController using modern, production-ready C++20 and facade architecture.
- Fixed 13 critical bugs (including memory safety leaks, data races, UI thread freezes, safety baseline sequences, speed-unit mismatch, premature homing completions, and startup races) to guarantee sound and reliable machine-safety and robust industrial operations.
- Build and deploy verified.