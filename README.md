# Agricultural Drone Telemetry & Adaptive Tank-Empty Detector — ROS 2

A ROS 2-based live telemetry visualizer and adaptive payload detector designed for autonomous agricultural spraying drones.

The system consumes **1 Hz agricultural drone telemetry through native ROS 2 bag playback**, visualizes the drone trajectory and synchronized telemetry signals, and implements an **adaptive mass-balance tank-empty detector** designed to prevent false Return-to-Launch (RTL) decisions caused by temporary flow interruptions.

## Project Overview

Agricultural spraying drones carry liquid payloads such as pesticides, herbicides, fungicides, or liquid fertilizers. Reliable estimation of remaining payload is important because an incorrect tank-empty decision can prematurely terminate a spraying mission.

This project addresses a telemetry scenario where the Companion Computer (CC) incorrectly declared the spray tank empty during a temporary flow interruption.

The implementation provides:

* Native ROS 2 bag playback
* ROS 2 telemetry subscriber
* Live trajectory visualization
* Synchronized telemetry plots
* Adaptive tank-volume estimation
* Flow-loss detection
* Temporal debouncing
* Sloshing/transient detection
* Genuine tank-empty confirmation
* Pump protection logic
* Comparison between the Companion Computer decision and the corrected detector

The detector operates using telemetry values rather than hard-coded timestamps or sample indices.

# Project Architecture

```text
agri_spray_ros2/
│
├── README.md
│   └── Complete project documentation and Task 2 report
│
├── requirements.txt
│   └── Python dependencies
│
├── ros2_live_visualizer.py
│   └── ROS 2 subscriber, detector and live visualizer
│
├── generate_ros2_bag.py
│   └── Generates a native ROS 2 bag from the telemetry CSV
│
├── spray_mission_assessment.csv
│   └── Original 1 Hz mission telemetry
│
└── spray_mission_bag/
    ├── spray_mission_bag_0.db3
    └── metadata.yaml
```

# System Data Flow

```text
                Mission Telemetry CSV
                         │
                         ▼
              generate_ros2_bag.py
                         │
                         ▼
                Native ROS 2 Bag
                         │
                         ▼
                  ros2 bag play
                         │
                         ▼
              ROS 2 Telemetry Topic
                         │
                         ▼
              ros2_live_visualizer.py
                    ┌────┴────┐
                    │         │
                    ▼         ▼
              Visualization  Detector
                    │         │
                    │         ▼
                    │    Tank Volume
                    │     Estimator
                    │         │
                    ▼         ▼
              Trajectory + Detector State
              Telemetry Time-Series Plots
```

---

# Requirements

## Operating System

Recommended:

* Ubuntu 26.04 LTS

## ROS 2

Supported distributions:

* ROS 2 lyrical

The example commands below use **ROS 2 lyrical**.

## Python

Python 3.10 or newer.

The following Python packages are required:

```text
PyYAML
matplotlib
```

The project also requires the Python packages provided by the ROS 2 installation.


# 1. Environment Setup

## Install ROS 2 and System Packages

If ROS 2 Humble is already installed, install the remaining dependencies:

```bash
sudo apt update

sudo apt install -y \
    ros-lyrical-ros-base \
    python3-pip \
    python3-dev \
    python3-tk
```

Source ROS 2:

```bash
source /opt/ros/lyrical/setup.bash
```


# 2. Python Virtual Environment

Create a virtual environment with access to the ROS 2 Python packages:

```bash
python3 -m venv ros2_env --system-site-packages
```

Activate it:

```bash
source ros2_env/bin/activate
```

Upgrade pip:

```bash
pip install --upgrade pip
```

Install project dependencies:

```bash
pip install -r requirements.txt
```


# 3. ROS 2 Bag

The repository contains a generated ROS 2 bag:

```text
spray_mission_bag/
```

Inspect the bag:

```bash
ros2 bag info spray_mission_bag
```

The bag contains the telemetry required by the visualization and tank-empty detection system.

---

# 4. Regenerating the ROS 2 Bag

The repository also contains:

```text
generate_ros2_bag.py
```

This script converts the telemetry CSV into a native ROS 2 bag.

Run:

```bash
source /opt/ros/humble/setup.bash

python3 main_ros2_writer.py your_telemetry_data.csv spray_mission_bag
```

The generated bag can then be inspected:

```bash
ros2 bag info spray_mission_bag
```

---

# 5. Running the System

Two terminals are recommended.

## Terminal 1 — Start the Visualizer

```bash
source /opt/ros/lyrical/setup.bash
source ros2_env/bin/activate

python3 ros2_live_visualizer.py
```

The node subscribes to the telemetry stream and processes the incoming messages.

---

## Terminal 2 — Play the ROS 2 Bag

```bash
source /opt/ros/lyrical/setup.bash

ros2 bag play spray_mission_bag
```

The telemetry is replayed through ROS 2 rather than being read directly from the CSV by the visualization node.

This is important because the visualization and detector operate on the same ROS 2 message stream that would be used during a live telemetry scenario.


# Visualization

The visualizer provides two main categories of information.

## 1. Drone Trajectory

The latitude and longitude telemetry is transformed into a local Cartesian coordinate representation.

The trajectory can therefore be displayed as:

```text
Y
↑
│          •
│       •     •
│    •           •
│  •               •
│
└──────────────────────→ X
```

This allows the flight path and spraying sections to be observed spatially.


## 2. Telemetry Time-Series

The visualizer synchronizes telemetry signals against mission time.

Important signals include:

* Flow rate
* Spray motor RPM
* Mission status
* Estimated remaining payload
* Companion Computer tank-empty decision
* Corrected detector state

This makes the false RTL event and the corrected detector response directly comparable.


# Input Telemetry

The mission telemetry contains fields including:

| Field              | Description                       |
| ------------------ | --------------------------------- |
| `time_ist`         | Mission timestamp in IST          |
| `lat`              | Latitude                          |
| `lon`              | Longitude                         |
| `heading_deg`      | Drone heading                     |
| `flow_sensor`      | Flow sensor pulse count           |
| `flow_rate_ml_min` | Measured liquid flow              |
| `spray_motor_rpm`  | Spray pump motor speed            |
| `status`           | Flight / Companion Computer state |

The telemetry is recorded at approximately **1 Hz**.

The detector does not assume that every interval is exactly 1 second.

Instead:

```text
Δt = t[n] - t[n-1]
```

is calculated dynamically for each sample.

This allows the estimator to handle occasional timing variations such as:

```text
0.8 s
1.0 s
1.2 s
```


# Task 2 — Spurious Tank-Empty Detection

## Problem

During the mission, the Companion Computer incorrectly declared the payload tank empty and initiated Return-to-Launch (RTL).

The problematic event occurred around:

```text
11:22:14 IST
```

At this point, the telemetry showed:

* Spray motor still active
* Flow rate near zero
* Approximately 4.5 L of payload still remaining

Therefore, an instantaneous low-flow test could not distinguish between:

1. Temporary flow interruption
2. Actual tank exhaustion
3. Pump/system fault
4. Air ingestion or sloshing


# Point 1 — Mechanism Behind the Spurious Tank-Empty Declaration

The original Companion Computer behavior can be represented conceptually as:

```text
IF:

    Spray Motor Active
        AND
    Flow Rate < Threshold

THEN:

    Declare TANK_EMPTY
        ↓
    Command RTL
```

This type of instantaneous decision is vulnerable to temporary flow interruptions.

During a banked turn, fluid movement inside the partially filled tank can cause:

* Liquid displacement
* Sloshing
* Temporary suction starvation
* Air entering the pump intake
* Temporary flow-sensor collapse

In the observed event, the flow measurement fell sharply while the spray motor continued operating.

The key problem was the absence of sufficient:

* Temporal debouncing
* Remaining-volume estimation
* State memory
* Recovery observation

Consequently, a short flow interruption was interpreted as complete tank exhaustion.


# Point 2 — Supporting Telemetry Samples

The telemetry around the event demonstrates why the instantaneous decision was incorrect.

| Timestamp | Flight Status       |  Motor Speed |  Flow Rate | Estimated Payload | Interpretation              |
| --------- | ------------------- | -----------: | ---------: | ----------------: | --------------------------- |
| 11:22:11  | `MISSION_SPRAYING`  |     2420 RPM | 460 mL/min |          ~4512 mL | Normal spraying             |
| 11:22:12  | `MISSION_SPRAYING`  |     2400 RPM |  12 mL/min |          ~4512 mL | Flow collapse begins        |
| 11:22:13  | `MISSION_SPRAYING`  |     2380 RPM |   0 mL/min |          ~4512 mL | Zero flow with active motor |
| 11:22:14  | `CC_RTL_TANK_EMPTY` | 2350 RPM → 0 |   0 mL/min |          ~4512 mL | Spurious RTL                |
| 11:22:15  | `RTL_RETURNING`     |        0 RPM |   0 mL/min |          ~4512 mL | Pump stopped                |

Approximately:

```text
4512 mL = 4.512 L
```

remained in the tank.

Relative to a 10 L initial payload:

```text
4512 / 10000 × 100 ≈ 45.1%
```

Thus, approximately **45% of the original payload remained** when the false tank-empty event occurred.


# Point 3 — Corrected Detector Design

The corrected detector combines:

1. Mass-balance volume estimation
2. Motor-state detection
3. Low-flow detection
4. Adaptive probe windows
5. Remaining-volume thresholds
6. Temporal confirmation

The detector therefore does not immediately equate:

```text
Low Flow = Empty Tank
```

Instead, it evaluates the situation as a state transition.


# 1. Mass-Balance State Estimator

The remaining payload is estimated using flow integration.

For each telemetry interval:

```text
ΔV = FlowRate / 60 × Δt
```

where:

```text
FlowRate = mL/min
Δt       = seconds
ΔV       = mL
```

The remaining volume is then:

```text
V_remaining[n] =
    V_remaining[n-1] - ΔV
```

with an initial condition:

```text
V_remaining = 10000 mL
```

Therefore:

```text
V_remaining(t)
=
V_initial
-
∫ FlowRate(t) / 60 dt
```


# Dynamic Time Integration

The implementation does not assume a fixed 1-second interval.

For every sample:

```text
Δt = timestamp[n] - timestamp[n-1]
```

Then:

```text
consumed_ml =
    flow_rate_ml_min / 60 × Δt
```

This is important because the log may contain intervals such as:

```text
0.8 s
1.0 s
1.2 s
```

Using the actual timestamp difference prevents integration error caused by assuming a constant sample interval.


# Refill Reset

The volume estimator supports resetting the estimated payload to the nominal capacity when an explicit ground-refill indication is present.

Conceptually:

```text
IF refill/reset event detected:

    V_remaining = 10000 mL
```

The detector therefore supports multiple flight/refill cycles without requiring a hard-coded mission timestamp.

---

# 2. Adaptive Dual-State Decision

The detector uses the following primary thresholds.

| Parameter                 |     Value |
| ------------------------- | --------: |
| Nominal tank capacity     | 10,000 mL |
| Motor RPM threshold       | 1,200 RPM |
| Low-flow threshold        | 50 mL/min |
| Slosh probe window        |     5.0 s |
| Empty-volume limit        |    300 mL |
| Empty confirmation window |     2.0 s |

---

# Motor RPM Threshold

```text
MOTOR_RPM_THRESH = 1200 RPM
```

Low-flow conditions are evaluated as a tank-empty candidate only when the spray motor is actually operating.

This avoids treating:

```text
Motor OFF + Flow = 0
```

as evidence of tank exhaustion.

---

# Low-Flow Threshold

```text
LOW_FLOW_THRESH = 50 mL/min
```

When:

```text
Motor RPM > 1200
```

and:

```text
Flow Rate < 50 mL/min
```

the detector enters a low-flow evaluation state.

It does not immediately declare the tank empty.


# Adaptive Probe Logic

## Case A — Significant Payload Remaining

If:

```text
V_remaining > 300 mL
```

then a low-flow event is treated as a possible transient condition.

The detector enters:

```text
PROBING_SLOSH
```

and starts a:

```text
5.0 second
```

probe window.

### If flow recovers

```text
Flow >= 50 mL/min
```

the detector:

```text
Cancel probe
Return to normal spraying
```

No tank-empty declaration is generated.

### If flow remains zero

If the low-flow condition persists beyond the probe window, the detector reports:

```text
FAULT_CLOG_OR_SYSTEM_ISSUE
```

rather than incorrectly assuming that the tank is empty.

This distinction is important because a substantial estimated payload remaining suggests that the problem may be:

* Sloshing
* Intake starvation
* Pump issue
* Clogged line
* Flow-sensor fault

rather than an empty tank.


# Case B — Payload Near Depletion

If:

```text
V_remaining <= 300 mL
```

the detector treats the situation as a genuine tank-empty candidate.

It enters:

```text
PROBING_TANK_EMPTY
```

and requires:

```text
2.0 seconds
```

of confirmation.

Only after the condition remains valid for the confirmation period does the detector produce:

```text
DETECTOR_TANK_EMPTY
```

This temporal confirmation prevents a single telemetry sample from triggering a tank-empty decision.

---

# Detector State Machine

The conceptual state machine is:

```text
                    Normal Flow
                       │
                       ▼
                DETECTOR_NOMINAL_OK
                       │
                       │ Motor > 1200 RPM
                       │ AND Flow < 50
                       ▼
                ┌───────────────┐
                │ Volume > 300  │
                └───────┬───────┘
                        │
                        ▼
                 PROBING_SLOSH
                    │       │
          Flow      │       │ No recovery
        recovers    │       ▼
                    │   FAULT_CLOG_
                    │   OR_SYSTEM_ISSUE
                    ▼
             DETECTOR_NOMINAL_OK


                Volume <= 300 mL
                       │
                       ▼
              PROBING_TANK_EMPTY
                       │
                2 s confirmation
                       │
                       ▼
             DETECTOR_TANK_EMPTY
```


# Why the 5-Second Slosh Probe?

The observed low-flow interruption lasted approximately:

```text
11:22:12 → 11:22:13+
```

while the tank still contained approximately:

```text
4.5 L
```

A temporal window allows the detector to distinguish between:

```text
Short transient
```

and:

```text
Persistent flow failure
```

The selected 5-second window is intended to tolerate temporary flow interruptions while still identifying persistent problems.


# Why the 300 mL Threshold?

The detector uses:

```text
EMPTY_VOLUME_LIMIT = 300 mL
```

as the near-empty reserve boundary.

This represents approximately:

```text
300 / 10000 × 100 = 3%
```

of the nominal 10 L tank capacity.

The threshold provides a distinction between:

```text
Substantial remaining payload
```

and:

```text
Near-empty payload condition
```

It also avoids using the raw flow signal alone to determine whether the tank has been depleted.


# Why the 2-Second Empty Confirmation?

A genuine tank-empty event should persist rather than appear for a single sample.

Therefore:

```text
Flow < 50 mL/min
AND
Motor RPM > 1200
AND
V_remaining <= 300 mL
```

must persist for approximately:

```text
2 seconds
```

before:

```text
DETECTOR_TANK_EMPTY
```

is generated.

This reduces sensitivity to one-sample sensor noise and protects the pump from continued operation after a confirmed depletion condition.


# Detector Parameters

```text
NOMINAL_CAPACITY_ML       = 10000.0
MOTOR_RPM_THRESH           = 1200.0
LOW_FLOW_THRESH_ML_MIN     = 50.0
SLOSH_PROBE_WINDOW_SEC     = 5.0
EMPTY_VOLUME_LIMIT_ML      = 300.0
EMPTY_CONFIRM_WINDOW_SEC   = 2.0
```

These values are explicitly defined rather than embedded as timestamp-specific conditions.


# Log-Wide Evaluation

| Mission Phase    | Timestamp           | Estimated Volume | CC Decision | Corrected Detector    |
| ---------------- | ------------------- | ---------------: | ----------- | --------------------- |
| Initial spraying | 11:00:00 – 11:22:11 |  10.0 L → 4.51 L | NOMINAL     | `DETECTOR_NOMINAL_OK` |
| Sloshing event   | 11:22:12 – 11:22:15 |          ~4.51 L | TANK_EMPTY  | `PROBING_SLOSH`       |
| Recovery         | 11:22:16 onward     | ~4.51 L → 0.30 L | NOMINAL     | `DETECTOR_NOMINAL_OK` |
| Tank exhaustion  | 11:38:40 – 11:38:43 |          <0.30 L | TANK_EMPTY  | `DETECTOR_TANK_EMPTY` |

The corrected detector therefore distinguishes the temporary low-flow event from the later genuine depletion event based on telemetry-derived state rather than timestamp-specific logic.


# False RTL Event

The problematic sequence was:

```text
Normal spraying
      │
      ▼
Flow decreases
      │
      ▼
Flow = 0
Motor still active
      │
      ▼
Instantaneous CC check
      │
      ▼
TANK_EMPTY
      │
      ▼
RTL
```

The corrected sequence is:

```text
Normal spraying
      │
      ▼
Flow decreases
      │
      ▼
Low-flow condition detected
      │
      ▼
Check estimated remaining volume
      │
      ▼
~4.5 L remaining
      │
      ▼
PROBING_SLOSH
      │
      ▼
Flow recovers
      │
      ▼
Return to nominal spraying
      │
      ▼
No tank-empty RTL
```


# Important Design Property

The detector contains **no hard-coded mission timestamps or sample indices**.

It does not use logic such as:

```python
if timestamp == "11:22:14":
    ignore_event()
```

or:

```python
if sample_index == 1234:
    tank_empty = False
```

Instead, decisions depend on telemetry-derived conditions:

```text
motor RPM
flow rate
estimated remaining volume
elapsed probe time
```

This makes the detector applicable to different mission logs and live telemetry streams.


# Handling Variable Sample Intervals

Although the source telemetry is nominally 1 Hz, actual intervals may vary.

For example:

```text
Sample 1 → Sample 2 = 1.0 s
Sample 2 → Sample 3 = 0.8 s
Sample 3 → Sample 4 = 1.2 s
```

The estimator uses:

```text
dt = current_timestamp - previous_timestamp
```

rather than assuming:

```text
dt = 1.0
```

This is particularly important for accurate payload integration.


# Secondary Telemetry Anomalies

The following observations were identified during telemetry analysis.

These are documented for completeness and are not modified by the tank-empty detector because the task is specifically scoped to payload detection.

## 1. Flow Startup Lag

At approximately:

```text
11:00:05 – 11:00:12 IST
```

the spray motor begins spinning before the flow rate reaches its normal operating value.

This is consistent with a pressure/build-up delay in the spraying system.

Therefore, startup flow should not necessarily be interpreted as a tank-empty condition.


## 2. Heading Wrapping

During pass turnarounds, heading measurements can exhibit approximately ±180° discontinuities.

For example:

```text
179°
-179°
```

represents a small physical heading change rather than a ~358° rotation.

A production trajectory-processing system should therefore normalize heading differences when calculating angular changes.


## 3. Variable Sample Timing

Most samples occur approximately every:

```text
1.0 second
```

but occasional intervals may be:

```text
0.8 seconds
1.2 seconds
```

The mass-balance estimator therefore uses the measured timestamp difference for every integration step.


# Design Assumptions

## Initial Payload

The detector assumes:

```text
Initial payload = 10,000 mL
```

unless an explicit refill/reset event is detected.


## Tank Capacity

The nominal tank capacity used by the estimator is:

```text
10 L
```

This is consistent with the mission scenario.


## Flow Measurement

The flow-rate telemetry is treated as:

```text
mL/min
```

and converted to mL/s during integration:

```text
flow_ml_per_second = flow_ml_per_minute / 60
```

## Telemetry Rate

The source telemetry is nominally:

```text
1 Hz
```

but the detector does not depend on a fixed sampling period.


# Reproducibility

Clone the repository:

```bash
git clone <YOUR_REPOSITORY_URL>
cd agri_spray_ros2
```

Create the environment:

```bash
source /opt/ros/lyrical/setup.bash

python3 -m venv ros2_env --system-site-packages
source ros2_env/bin/activate

pip install -r requirements.txt
```

Inspect the bag:

```bash
ros2 bag info spray_mission_bag
```

Start the visualizer:

```bash
python3 ros2_live_visualizer.py
```

In another terminal:

```bash
source /opt/ros/lyrical/setup.bash

ros2 bag play spray_mission_bag
```


# Verification Checklist

After starting the system, verify the following:

* [ ] ROS 2 environment is sourced
* [ ] Python environment is activated
* [ ] ROS 2 bag is recognized
* [ ] Telemetry messages are being published
* [ ] Visualizer receives telemetry
* [ ] Trajectory is displayed
* [ ] Flow-rate plot is displayed
* [ ] Motor RPM plot is displayed
* [ ] Remaining payload is calculated
* [ ] The 11:22:12–11:22:14 low-flow event is detected
* [ ] The detector enters `PROBING_SLOSH`
* [ ] No corrected tank-empty event is generated during the false RTL event
* [ ] Flow recovery returns the detector to nominal state
* [ ] The later genuine low-volume event is detected
* [ ] `DETECTOR_TANK_EMPTY` is generated after confirmation

# Key Technical Concepts Demonstrated

This project demonstrates practical concepts in:

* ROS 2
* ROS 2 bag playback
* ROS 2 publishers/subscribers
* Python ROS 2 nodes
* Telemetry processing
* Sensor data integration
* State-machine design
* Time-based debouncing
* Adaptive thresholding
* Mass-balance estimation
* Flow-rate processing
* Embedded/control-system fault detection
* Pump protection
* Real-time visualization
* Mission telemetry analysis
* Cartesian trajectory visualization
* Handling irregular telemetry timing


# Task 2 Conclusion

The telemetry demonstrates that the Companion Computer's tank-empty declaration around **11:22:14 IST** occurred while approximately **4.5 L of payload remained**.

The failure mode was an instantaneous interpretation of:

```text
Active spray motor
+
Near-zero flow
=
Tank empty
```

The corrected detector introduces state, time, and estimated remaining payload.

Its decision process is:

```text
Motor state
     +
Flow state
     +
Estimated remaining volume
     +
Temporal confirmation
     ↓
Detector state
```

This allows temporary flow interruptions to be distinguished from genuine payload depletion.

The approach also avoids hard-coded timestamps and sample indices, allowing the detector to operate on different telemetry sequences using the same underlying detection logic.


# Repository Contents

```text
agri_spray_ros2/
│
├── README.md
├── requirements.txt
├── ros2_live_visualizer.py
├── generate_ros2_bag.py
├── spray_mission_assessment.csv
│
└── spray_mission_bag/
    ├── spray_mission_bag_0.db3
    └── metadata.yaml
```

# Author

**Abhirama M.**

Agricultural Drone Telemetry & Adaptive Tank-Empty Detector
ROS 2 Assessment Project
