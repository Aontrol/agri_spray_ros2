# Agricultural Drone Telemetry & Adaptive Tank-Empty Detector — ROS 2

 A ROS 2-based live telemetry visualizer and adaptive payload detector designed for autonomous agricultural spraying drones.

 The system consumes **1 Hz agricultural drone telemetry** through native ROS 2 bag playback, visualizes the drone trajectory and synchronized telemetry signals in real time, and implements an **adaptive mass-balance tank-empty detector** designed to prevent false Return-to-Launch (RTL) decisions caused by temporary flow interruptions during flight.


 ## Project Overview

 Agricultural spraying drones carry liquid payloads such as pesticides, herbicides, fungicides, or liquid fertilizers. Accurate payload estimation is critical because an incorrect tank-empty declaration can prematurely terminate a spraying mission and cause an unnecessary Return-to-Launch maneuver.

 This project addresses a telemetry scenario where the onboard **Companion Computer (CC)** incorrectly declared the spray tank empty during a temporary flow interruption, leaving more than **4.5 L (45%) of usable liquid** in the tank.



 ## Key Features

 - **Native ROS 2 Bag Generation**
  - Converts raw 1 Hz CSV telemetry into SQLite3 ROS 2 bag files.
  - Uses standard ROS 2 message definitions:
    - `PoseStamped`
    - `Float32`
    - `String`
- **Live Telemetry Stream Processing**
  - Consumes ROS 2 bag topics through `ros2 bag play`.
  - Processes telemetry in real time through a ROS 2 subscriber node.
- **Real-Time Visualization Dashboard**
  - Displays the drone trajectory in a local East-North-Up (ENU) coordinate frame.
  - Displays synchronized time-series telemetry plots.
  - Visualizes:
    - Flow rate
    - Motor RPM
    - Estimated payload volume
    - Flight state
- **Adaptive Mass-Balance Detector**
  - Estimates remaining payload using flow-rate integration.
  - Uses dynamic time intervals:
     $\Delta V = \text{FlowRate} \times \Delta t$
  - Includes a **5-second slosh-probe window**.
  - Includes a **2-second empty-confirmation window**.
  - Prevents temporary flow interruptions from triggering false RTL decisions.
  - Protects the pump from prolonged operation when the tank is genuinely empty.



 ## Repository Structure

```
agri_spray_ros2/
│
├── README.md                     # Project documentation and Task 2 report
├── requirements.txt              # Python dependencies
├── generate_ros2_bag.py          # Standalone CSV → ROS 2 bag converter
├── ros2_live_visualizer.py       # ROS 2 subscriber, detector node & GUI dashboard
├── spray_mission_assessment.csv  # Raw 1 Hz mission telemetry input
│
└── spray_mission_bag/            # Generated ROS 2 bag directory
    ├── spray_mission_bag_0.db3
    └── metadata.yaml
```


 # How to Build and Run

 ## Prerequisites

 | Requirement | Supported Versions |
| --- | --- |
| Operating System | Ubuntu 22.04 / 24.04 / 26.04 LTS |
| ROS 2 | Humble / Iron / Jazzy / Lyrical |
| Python | 3.10+ |
| GUI | `python3-tk` |

A ROS 2 `ros-base` installation is required.


 ## Step 1 — System Dependencies & Environment Setup

 Source your installed ROS 2 distribution:

```
source /opt/ros/lyrical/setup.bash
```

 > Replace `lyrical` with your installed ROS 2 distribution, for example `humble` or `jazzy`.

 Create a Python virtual environment while retaining access to the system ROS 2 Python packages:

```
python3 -m venv ros2_env --system-site-packages
```

 Activate the environment:

```
source ros2_env/bin/activate
```

 Install the Python dependencies:

```
pip install --upgrade pip
pip install -r requirements.txt
```


 # Step 2 — Generate the Native ROS 2 Bag

 Convert the telemetry CSV file into a native ROS 2 bag:

```
python3 generate_ros2_bag.py \
    spray_mission_assessment.csv \
    spray_mission_bag
```

 Inspect the generated bag metadata:

```
ros2 bag info spray_mission_bag
```

 The generated bag uses the SQLite3 storage backend and can be replayed using the standard ROS 2 CLI.


 # Step 3 — Run the Visualizer and Play the Bag

 Open **two separate terminals**.

 ## Terminal 1 — Start the ROS 2 Subscriber and Dashboard

 Source ROS 2:

```
source /opt/ros/lyrical/setup.bash
```

 Activate the Python environment:

```
source ros2_env/bin/activate
```

 Start the visualizer:

```
python3 ros2_live_visualizer.py
```

 The GUI dashboard will open and wait for the live ROS 2 telemetry stream.


 ## Terminal 2 — Play the ROS 2 Bag

 Source ROS 2:

```
source /opt/ros/lyrical/setup.bash
```

 Play the telemetry bag:

```
ros2 bag play spray_mission_bag
```

 The telemetry will be replayed in real time at the original **1 Hz sampling rate** and processed live by the ROS 2 node.


 # Task 2 — Spurious Tank-Empty Detection Analysis

 ## Point 1 — Mechanism Behind the Spurious Tank-Empty Declaration

 The original onboard Companion Computer (CC) used an instantaneous, stateless boolean rule:

 $$
\text{Pump Active (RPM > Threshold)}
\land
\text{Flow Rate < Threshold}
\implies
\text{TANK\_EMPTY}
\implies
\text{RTL}
$$

 During pass turnarounds or banked maneuvers, centrifugal forces and liquid movement inside the partially filled tank can cause:

 - Liquid displacement away from the bottom suction port.
- Temporary air ingestion into the pump inlet.
- Momentary collapse of the inline rotor flow-sensor output.

 Because the original CC had:

 - No state memory.
- No temporal debouncing.
- No mass-balance estimation.

 a **1–2 second flow interruption** during pass alignment could immediately be interpreted as complete tank exhaustion.

 This resulted in an unnecessary RTL command even though a substantial quantity of liquid remained in the tank.


 # Point 2 — Supporting Telemetry Samples from the Log

 Telemetry around **11:22:12–11:22:14 IST** shows the failure sequence.

| Timestamp (IST) | Flight Status | Motor Speed (RPM) | Flow Rate (mL/min) | Estimated Payload | Physical Interpretation |
| --- | --- | --- | --- | --- | --- |
| 11:22:11 | `MISSION_SPRAYING` | 2420 | 460 | \~4.51 L | Normal spraying operation |
| 11:22:12 | `MISSION_SPRAYING` | 2400 | 12 | \~4.51 L | Pass turnaround; flow collapses |
| 11:22:13 | `MISSION_SPRAYING` | 2380 | 0 | \~4.51 L | Zero flow with active motor; possible air pocket/slosh |
| 11:22:14 | `CC_RTL_TANK_EMPTY` | 2350 → 0 | 0 | \~4.51 L | Spurious RTL triggered by CC |
| 11:22:15 | `RTL_RETURNING` | 0 | 0 | \~4.51 L | Pump disengaged; drone returning to base |

### Key Evidence

 At the exact second when `CC_RTL_TANK_EMPTY` was declared, approximately:

 **4.512 L of liquid remained in the nominal 10 L tank.**

 That corresponds to approximately:

 $$
\frac{4512}{10000}\times100 \approx 45.1\%
$$

 of the nominal tank capacity.

 Therefore, the instantaneous low-flow condition did not represent genuine tank depletion.


 # Point 3 — Corrected Detector Design & Log-Wide Evaluation

 The corrected detector combines:

 1. **Mass-balance volume tracking**
2. **Adaptive temporal probing**
3. **Near-empty confirmation**
4. **Explicit detector states**

 ## Detector State Machine

```
                           Normal Flow
                                │
                                ▼
                    ┌────────────────────────┐
                    │ DETECTOR_NOMINAL_OK    │
                    └────────────┬───────────┘
                                 │
                                 │ Motor > 1200 RPM
                                 │ AND Flow < 50 mL/min
                                 ▼
                       ┌───────────────────┐
                       │ Volume > 300 mL ? │
                       └─────────┬─────────┘
                                 │
                 ┌───────────────┴────────────────┐
                 │                                │
                 ▼                                ▼
        Volume > 300 mL                    Volume ≤ 300 mL
                 │                                │
                 ▼                                ▼
        ┌──────────────────┐             ┌──────────────────────┐
        │ PROBING_SLOSH    │             │ PROBING_TANK_EMPTY   │
        │ 5 s Probe Window │             │ 2 s Confirm Window   │
        └────────┬─────────┘             └──────────┬───────────┘
                 │                                  │
          ┌──────┴───────┐                    2 s Timer
          │              │                    Expired
       Flow Recovers   5 s Timer                  │
          │            Expired                    ▼
          ▼              │              ┌──────────────────────┐
 DETECTOR_NOMINAL_OK     ▼              │ DETECTOR_TANK_EMPTY  │
                    FAULT_CLOG_OR_      └──────────────────────┘
                    LINE_ISSUE
```


 # Thresholds & Time Constants

 | Parameter | Value | Purpose |
| --- | --- | --- |
| Nominal Tank Capacity | 10,000 mL | Initial payload capacity |
| Motor RPM Threshold | 1,200 RPM | Determines whether the pump is active |
| Low-Flow Threshold | 50 mL/min | Detects suction loss or flow blockage |
| Slosh Probe Window | 5.0 s | Allows temporary flow interruptions to recover |
| Empty Reserve Boundary | 300 mL | Near-empty threshold (\~3% capacity) |
| Empty Confirmation Window | 2.0 s | Confirms persistent low flow before tank-empty declaration |


 ## Mass-Balance Volume Estimation

 The detector estimates payload volume by integrating the measured flow rate over the actual telemetry time interval.

 For each telemetry sample:

 $$
\Delta V = Q \times \Delta t
$$

 where:

 - $\\Delta V$ = volume consumed
- $Q$ = measured flow rate
- $\\Delta t$ = elapsed time between telemetry samples

 The remaining payload is then updated as:

 $$
V_{remaining,n}
=
V_{remaining,n-1}
-
Q_n\Delta t
$$

 The implementation uses:

 $$
\Delta t = t_n - t_{n-1}
$$

 rather than assuming every telemetry sample occurs exactly one second apart.

 This makes the detector robust to small variations in telemetry timing.


 # Detector Logic

 ## 1\. Nominal Operation

 When the pump is active and measured flow is above the low-flow threshold:

```
Motor RPM > 1200
AND
Flow Rate >= 50 mL/min
```

 the detector remains in:

```
DETECTOR_NOMINAL_OK
```

 The estimated payload volume continues to be updated through mass-balance integration.


 ## 2\. Slosh Probe

 If:

```
Motor RPM > 1200
AND
Flow Rate < 50 mL/min
AND
Estimated Volume > 300 mL
```

 the detector enters:

```
PROBING_SLOSH
```

 The detector waits for up to **5 seconds**.

 ### If flow recovers

 The detector returns to:

```
DETECTOR_NOMINAL_OK
```

 This prevents a temporary suction interruption caused by tank sloshing from triggering an RTL.

 ### If flow does not recover

 The event is classified as:

```
FAULT_CLOG_OR_LINE_ISSUE
```

 rather than immediately declaring the tank empty.


 ## 3\. Tank-Empty Confirmation

 If:

```
Motor RPM > 1200
AND
Flow Rate < 50 mL/min
AND
Estimated Volume <= 300 mL
```

 the detector enters:

```
PROBING_TANK_EMPTY
```

 A **2-second confirmation timer** is started.

 Only if the low-flow condition persists for the full confirmation period does the detector issue:

```
DETECTOR_TANK_EMPTY
```

 This reduces the risk of pump dry-running while avoiding an instantaneous empty declaration.


 # Log-Wide Verdict Comparison

 | Mission Phase | Timestamp (IST) | Estimated Volume | CC Decision | Corrected Detector Decision | Justification |
| --- | --- | --- | --- | --- | --- |
| Initial Spraying | 11:00:00 – 11:22:11 | 10.0 L → 4.51 L | `NOMINAL` | `DETECTOR_NOMINAL_OK` | Normal operation |
| Pass Turnaround Slosh | 11:22:12 – 11:22:15 | \~4.51 L | `TANK_EMPTY` | `PROBING_SLOSH` | False alarm suppressed; volume \> 300 mL |
| Mission Resume | 11:22:16 – 11:38:39 | 4.51 L → 0.30 L | `NOMINAL` | `DETECTOR_NOMINAL_OK` | Continuous spraying and payload tracking |
| Genuine Depletion | 11:38:40 – 11:38:43 | \<0.30 L | `TANK_EMPTY` | `DETECTOR_TANK_EMPTY` | Genuine empty condition confirmed after 2 s |


 # System Assumptions

 ## Initial Payload

 The tank is assumed to be filled to the nominal:

```
10,000 mL
```

 capacity before takeoff.

 The payload estimate can be reset when a suitable `REFILL` or `GROUND_RESET` status message is detected.


 ## Flow Sensor Calibration

 The telemetry field:

```
flow_rate_ml_min
```

 is assumed to be linearly calibrated and expressed in:

```
mL/min
```


 ## Sampling Integration

 Telemetry is integrated using dynamic time intervals:

 $$
\Delta t = t_n - t_{n-1}
$$

 rather than assuming a rigid 1.0-second sample interval.


 ## Coordinate Frame

 Global GNSS coordinates (`Latitude`, `Longitude`) are converted into a local planar **East-North-Up (ENU)** coordinate frame.

 The first valid GPS position is used as the local reference point.

 Conceptually:

```
                 North (N)
                    ▲
                    │
                    │
                    │
                    └──────────► East (E)

                 Up (U)
                   ▲
                   │
                   │
                   ● Reference
                     Position
```

 This allows the dashboard to display the drone trajectory in meters rather than raw latitude/longitude coordinates.


 # Architecture

```
                  spray_mission_assessment.csv
                              │
                              ▼
                    ┌────────────────────┐
                    │ generate_ros2_bag  │
                    │      .py           │
                    └─────────┬──────────┘
                              │
                              ▼
                    Native ROS 2 SQLite Bag
                              │
                              │ ros2 bag play
                              ▼
                  ┌────────────────────────┐
                  │     ROS 2 Topics       │
                  │                        │
                  │ PoseStamped            │
                  │ Float32                │
                  │ String                 │
                  └────────────┬───────────┘
                               │
                               ▼
                  ┌────────────────────────┐
                  │ ros2_live_visualizer   │
                  │         .py            │
                  │                        │
                  │ ROS 2 Subscriber       │
                  │ Mass-Balance Estimator │
                  │ Adaptive Detector      │
                  │ State Machine          │
                  │ GUI Dashboard          │
                  └────────────┬───────────┘
                               │
                    ┌──────────┴──────────┐
                    ▼                     ▼
             Trajectory Plot        Telemetry Plots
                    │                     │
                    └──────────┬──────────┘
                               ▼
                       Live Visualization
```


 # Why the Adaptive Detector Matters

 The central failure mode addressed by this project is the difference between:

 > **Temporary loss of measurable flow**

 and

 > **Actual absence of liquid payload**

 An instantaneous threshold cannot reliably distinguish these two conditions.

 The adaptive detector instead combines:

 - Pump state
- Flow rate
- Estimated remaining volume
- Temporal persistence
- Slosh recovery time
- Empty confirmation

 This allows the system to treat low-flow events differently depending on the estimated payload remaining in the tank.

 When significant payload remains, the detector allows additional time for the flow to recover. When the tank is genuinely near empty, the detector uses a shorter confirmation period to protect the pump.


 # Results

 The telemetry analysis demonstrates that the original instantaneous detector could generate an RTL decision while approximately **4.512 L** of liquid remained.

 The corrected detector identifies the same event as a temporary slosh-related low-flow condition because:

```
Estimated Payload ≈ 4.51 L
              >
Empty Boundary = 0.30 L
```

 The event therefore enters the slosh-probing state rather than immediately declaring the tank empty.

 Later, when the estimated payload falls below the **300 mL reserve boundary** and the low-flow condition persists for the required **2 seconds**, the detector transitions to:

```
DETECTOR_TANK_EMPTY
```

 This provides a stateful and mass-balance-aware alternative to the original instantaneous tank-empty rule.


 # Technologies Used

 - **ROS 2**
- **Python 3**
- **SQLite3 ROS 2 Bags**
- **ROS 2 `rosbag2`**
- **Tkinter**
- **Matplotlib**
- **NumPy**
- **CSV telemetry processing**
- **ENU coordinate transformation**
- **Finite-state detection logic**
- **Mass-balance payload estimation**


 # Running the Project

 The complete workflow is:

```
# 1. Source ROS 2
source /opt/ros/lyrical/setup.bash

# 2. Activate environment
source ros2_env/bin/activate

# 3. Generate ROS 2 bag
python3 generate_ros2_bag.py \
    spray_mission_assessment.csv \
    spray_mission_bag

# 4. Start visualizer
python3 ros2_live_visualizer.py
```

 In another terminal:

```
source /opt/ros/lyrical/setup.bash

ros2 bag play spray_mission_bag
```

 # Author

 **Abhirama M.**

 GitHub: https://github.com/Aontrol
