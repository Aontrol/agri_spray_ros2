# Agricultural Drone Telemetry & Adaptive Tank-Empty Detector — ROS 2

A ROS 2-based live telemetry visualizer and adaptive payload detector designed for autonomous agricultural spraying drones.

The system consumes **1 Hz agricultural drone telemetry through native ROS 2 bag playback**, visualizes the drone trajectory and synchronized telemetry signals in real time, and implements an **adaptive mass-balance tank-empty detector** designed to prevent false Return-to-Launch (RTL) decisions caused by temporary flow interruptions during flight.

## Project Overview

Agricultural spraying drones carry liquid payloads (pesticides, herbicides, fungicides, or liquid fertilizers). Accurate payload estimation is critical because an incorrect tank-empty declaration prematurely terminates a spraying mission and causes unnecessary return-to-launch maneuvers.

This project addresses a telemetry scenario where the onboard Companion Computer (CC) incorrectly declared the spray tank empty during a temporary flow interruption, leaving over 4.5 L (45%) of usable liquid in the tank.

### Key Features

* **Native ROS 2 Bag Generation**: Converts raw 1 Hz CSV telemetry into SQLite3 ROS 2 bag files using standard ROS 2 message definitions (`PoseStamped`, `Float32`, `String`).

* **Live Telemetry Stream Processing**: Consumes ROS 2 bag topics via `ros2 bag play` in real time.

* **Real-Time Visualization Dashboard**: Renders local East-North-Up (ENU) trajectory maps alongside synchronized time-series telemetry plots (flow rate, motor RPM, estimated payload volume).

* **Adaptive Mass-Balance Detector**: Integrates flow rates dynamically ($\Delta V = \text{FlowRate} \times \Delta t$) with a 5-second slosh-probe window and a 2-second empty confirmation window to prevent false RTL triggers while protecting the pump.

## Repository Structure

```
agri_spray_ros2/
│
├── README.md                     # Project documentation and Task 2 report
├── requirements.txt              # Python dependencies
├── generate_ros2_bag.py          # Standalone CSV to ROS 2 bag converter
├── ros2_live_visualizer.py       # ROS 2 subscriber, detector node & live GUI dashboard
├── spray_mission_assessment.csv  # Raw 1 Hz mission telemetry input
│
└── spray_mission_bag/            # Generated ROS 2 bag directory
    ├── spray_mission_bag_0.db3
    └── metadata.yaml

```

## How to Build and Run (End-to-End)

### Prerequisites

* **OS**: Ubuntu 22.04 LTS / 24.04 LTS / 26.04 LTS

* **ROS 2**: Humble, Iron, Jazzy, or Lyrical (`ros-*-ros-base` installed)

* **Python**: 3.10+ with `python3-tk` installed

### Step 1: System Dependencies & Environment Setup

1. **Source your installed ROS 2 distribution**:

   ```
   source /opt/ros/lyrical/setup.bash
   # (Replace 'lyrical' with your ROS 2 distro, e.g., humble/jazzy)
   
   ```

2. **Create a Python virtual environment with system site-packages**:

   ```
   python3 -m venv ros2_env --system-site-packages
   source ros2_env/bin/activate
   
   ```

3. **Install Python dependencies**:

   ```
   pip install --upgrade pip
   pip install -r requirements.txt
   
   ```

### Step 2: Generate the Native ROS 2 Bag

Run the generator script to convert the telemetry CSV into a native ROS 2 bag:

```
python3 generate_ros2_bag.py spray_mission_assessment.csv spray_mission_bag

```

Inspect the generated bag metadata:

```
ros2 bag info spray_mission_bag

```

### Step 3: Run the Visualizer and Play the Bag

Open **two separate terminals**:

#### Terminal 1 — Start the ROS 2 Subscriber Node & Dashboard

```
source /opt/ros/lyrical/setup.bash
source ros2_env/bin/activate

python3 ros2_live_visualizer.py

```

*The visualizer GUI dashboard will open and wait for the live ROS 2 topic stream.*

#### Terminal 2 — Play the ROS 2 Bag

```
source /opt/ros/lyrical/setup.bash

ros2 bag play spray_mission_bag

```

*The telemetry will be replayed in real time (1 Hz) and processed live by the node.*

## Task 2 — Spurious Tank-Empty Detection Analysis

### Point 1: Mechanism Behind the Spurious Tank-Empty Declaration

The onboard Companion Computer (CC) evaluated an instantaneous, stateless boolean rule:

$$
\text{Pump Active (RPM > Threshold)} \land \text{Flow Rate < Threshold} \implies \text{TANK\_EMPTY} \implies \text{RTL}
$$

During pass turnarounds or banked maneuvers, centrifugal forces and liquid movement in the partially filled tank cause:

1. Liquid displacement away from the bottom suction port.

2. Temporary air ingestion into the pump inlet.

3. Momentary collapse of the inline rotor flow sensor output.

Because the CC possessed zero state memory, zero temporal debouncing, and no mass-balance estimation, a 1-to-2 second flow interruption during pass alignment was instantly classified as complete tank exhaustion, triggering an unnecessary RTL.

### Point 2: Supporting Telemetry Samples from the Log

Examining the logged telemetry around **11:22:12 – 11:22:14 IST** reveals the exact failure sequence:

| Timestamp (IST) | Flight Status | Motor Speed (RPM) | Flow Rate (mL/min) | Estimated Payload | Physical Interpretation | 
 | ----- | ----- | ----- | ----- | ----- | ----- | 
| **11:22:11** | `MISSION_SPRAYING` | 2420 | 460 | \~4.51 L | Normal spraying operation | 
| **11:22:12** | `MISSION_SPRAYING` | 2400 | 12 | \~4.51 L | Pass turnaround maneuver; flow collapses | 
| **11:22:13** | `MISSION_SPRAYING` | 2380 | 0 | \~4.51 L | Zero flow with active motor (air pocket/slosh) | 
| **11:22:14** | `CC_RTL_TANK_EMPTY` | 2350 → 0 | 0 | **\~4.51 L** | **Spurious RTL triggered by CC!** | 
| **11:22:15** | `RTL_RETURNING` | 0 | 0 | \~4.51 L | Pump disengaged; drone returning to base | 

**Key Evidence**: At the exact second `CC_RTL_TANK_EMPTY` was declared, approximately **4.512 L of liquid remained** in the 10 L tank (45.1% full).

### Point 3: Corrected Detector Design & Log-Wide Evaluation

The corrected detector implements a **dual-state machine** combining **mass-balance volume tracking** with **adaptive temporal probing**:

```
                           Normal Flow
                                │
                                ▼
                       DETECTOR_NOMINAL_OK
                                │
                                │ Motor > 1200 RPM
                                │ AND Flow < 50 mL/min
                                ▼
                       ┌─────────────────┐
                       │ Volume > 300 mL │
                       └────────┬────────┘
                                │
             ┌──────────────────┴──────────────────┐
             ▼                                     ▼
      [Volume > 300 mL]                     [Volume ≤ 300 mL]
             │                                     │
             ▼                                     ▼
       PROBING_SLOSH                       PROBING_TANK_EMPTY
       (5s Probe Window)                   (2s Confirm Window)
        │             │                            │
  Flow  │             │ 5s Timer                   │ 2s Timer
recovers│             │ Expired                    │ Expired
        ▼             ▼                            ▼
  DETECTOR_   FAULT_CLOG_OR_LINE_             DETECTOR_
  NOMINAL_OK      ISSUE                          TANK_EMPTY

```

#### Thresholds & Time Constants

1. **Nominal Tank Capacity (`10,000 mL`)**: Initialized at takeoff; resets upon detecting `REFILL`/`GROUND_RESET`.

2. **Motor RPM Threshold (`1,200 RPM`)**: Ensures low-flow logic only evaluates when the pump is commanded ON.

3. **Low-Flow Threshold (`50 mL/min`)**: Cutoff for detecting suction loss or flow blockages.

4. **Slosh Probe Window (`5.0 s`)**: Allows temporary sloshing or line pressure lag to self-correct when estimated payload is high ($> 300\text{ mL}$).

5. **Empty Reserve Boundary (`300 mL`)**: Defines the near-empty threshold (\~3% capacity).

6. **Confirmation Window (`2.0 s`)**: Requires a low-flow condition to persist for 2 seconds when volume $\le 300\text{ mL}$ before issuing `DETECTOR_TANK_EMPTY` to save the pump from running dry.

#### Log-Wide Verdict Comparison

| Mission Phase | Timestamp (IST) | Est. Volume | CC Decision | Corrected Detector Decision | Justification | 
 | ----- | ----- | ----- | ----- | ----- | ----- | 
| Initial Spraying | 11:00:00 – 11:22:11 | 10.0L → 4.51L | `NOMINAL` | `DETECTOR_NOMINAL_OK` | Normal operation. | 
| **Pass Turnaround Slosh** | **11:22:12 – 11:22:15** | **\~4.51L** | **`TANK_EMPTY`** | **`PROBING_SLOSH`** | **False alarm suppressed; volume > 300 mL.** | 
| Mission Resume | 11:22:16 – 11:38:39 | 4.51L → 0.30L | `NOMINAL` | `DETECTOR_NOMINAL_OK` | Continuous spraying tracking. | 
| **Genuine Depletion** | **11:38:40 – 11:38:43** | **<0.30L** | `TANK_EMPTY` | `DETECTOR_TANK_EMPTY` | **Genuine empty confirmed after 2s delay.** | 

## System Assumptions

1. **Initial Payload**: Tank is filled to nominal 10,000 mL capacity prior to flight start unless reset via status message.

2. **Flow Sensor Calibration**: Telemetry `flow_rate_ml_min` is assumed linearly calibrated in mL/min.

3. **Sampling Integration**: Integrates volume using dynamic step sizes ($\Delta t = t_n - t_{n-1}$) rather than assuming rigid 1.0s sample steps.

4. **Coordinate Frame**: Global GNSS coordinates (Lat/Lon) are mapped to a local planar East-North-Up (ENU) frame in meters relative to the first valid GPS lock.

## Author

* **Abhirama M.** — [*GitHub Profile*](https://github.com/Aontrol?utm_source=gemini)
