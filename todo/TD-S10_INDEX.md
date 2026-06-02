# TD-S10 Todo Index

This index maps the TD-S10 planning documents to their implementation or
research follow-ups.

## Planning / Objective Documents

| ID | File | Summary |
| --- | --- | --- |
| A1 | [TD-S10-A1_create_synthetic_dataset_whycon.md](TD-S10-A1_create_synthetic_dataset_whycon.md) | Define the synthetic Gazebo/WhyCon dataset objective. |
| A2 | [TD-S10-A2_validate_synthetic_whycon.md](TD-S10-A2_validate_synthetic_whycon.md) | Define validation of WhyCon on synthetic data. |
| A3 | [TD-S10-A3_calibrate_takeoff_camera_orientation.md](TD-S10-A3_calibrate_takeoff_camera_orientation.md) | Define takeoff-time camera orientation calibration. |
| A4 | [TD-S10-A4_inflight_handoff_hl_marker_control.md](TD-S10-A4_inflight_handoff_hl_marker_control.md) | Define in-flight handoff to estimator-backed marker control. |
| A5 | [TD-S10-A5_migrate_calib_marker_control_to_cpp_tasks.md](TD-S10-A5_migrate_calib_marker_control_to_cpp_tasks.md) | Define migration of calibration and marker control into C++ runtime tasks. |
| A6 | [TD-S10-A6_define_mission_level_robot_primitives.md](TD-S10-A6_define_mission_level_robot_primitives.md) | Define SentAI primitive taxonomy, REPL/radio grammar, and mission vocabulary. |
| A7 | [TD-S10-A7_runtime_namespace_alignment.md](TD-S10-A7_runtime_namespace_alignment.md) | Define runtime namespace alignment with the A6/B6 taxonomy, without breaking B5. |
| A8 | [TD-S10-A8_research_arm_emulator_runtime.md](TD-S10-A8_research_arm_emulator_runtime.md) | Research moving SIM proof work from FreeRTOS POSIX/x86 toward ARM firmware emulation. |

## Implementation / Research Documents

| ID | File | Summary |
| --- | --- | --- |
| B1 | [TD-S10-B1_implement_synthetic_dataset_whycon.md](TD-S10-B1_implement_synthetic_dataset_whycon.md) | Implement the A1 synthetic WhyCon dataset generator. |
| B2 | [TD-S10-B2_implement_validate_synthetic_whycon.md](TD-S10-B2_implement_validate_synthetic_whycon.md) | Implement A2 synthetic WhyCon validation. |
| B3 | [TD-S10-B3_implement_calibrate_takeoff_camera_orientation.md](TD-S10-B3_implement_calibrate_takeoff_camera_orientation.md) | Implement A3 takeoff camera orientation calibration. |
| B4 | [TD-S10-B4_implement_inflight_handoff_hl_marker_control.md](TD-S10-B4_implement_inflight_handoff_hl_marker_control.md) | Implement A4 in-flight marker-control handoff. |
| B5 | [TD-S10-B5_implement_cpp_tasks_for_calib_marker_control.md](TD-S10-B5_implement_cpp_tasks_for_calib_marker_control.md) | Implement A5 C++ runtime tasks for calibration and marker control. |
| B6 | [TD-S10-B6_research_sentai_prime_command_dictionary.md](TD-S10-B6_research_sentai_prime_command_dictionary.md) | Research A6 against SOTA and define D1-D8 conclusions for command taxonomy. |
| B7 | [TD-S10-B7_implement_runtime_namespace_alignment.md](TD-S10-B7_implement_runtime_namespace_alignment.md) | Implement A7 runtime namespace inventory, SIM/ARM parity checks, and safe introspection. |
| B8 | [TD-S10-B8_research_arm_emulator_runtime.md](TD-S10-B8_research_arm_emulator_runtime.md) | Research and spike ARM/Cortex-M emulator runtime path for SentAI. |

## Current Position

A6/B6 establish the language and architecture.  A7/B7 aligned much of the real
`sentai.*` runtime namespace surface, but B7 is paused after the POSIX/x86 SIM
path hit task/peripheral-fidelity limits.  A8/B8 now research a closer ARM
emulator path that can run the ARM FreeRTOS/task/ISR model on the host.
