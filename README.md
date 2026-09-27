# ESP32 LiDAR SLAM Robot

A custom differential-drive mobile robot built around an **ESP32**, **RPLIDAR A1**, wheel encoders, and a PC-based 2D SLAM pipeline.

The ESP32 handles real-time hardware interaction, motor control, sensor acquisition, and networking. LiDAR and wheel-encoder telemetry are streamed over UDP to a host computer, where Python code reconstructs LiDAR scans, estimates robot motion, performs scan matching, builds an occupancy grid, and visualizes the result using Rerun.

The project is built largely from the ground up rather than using an existing robotics or SLAM framework.

---

## Overview

The system is split into two main parts:

```text
┌──────────────────────────── Robot ────────────────────────────┐
│                                                              │
│                         ESP32                                │
│                           │                                  │
│        ┌──────────────────┼────────────────────┐             │
│        │                  │                    │             │
│   Motor Control      Wheel Encoders        RPLIDAR           │
│      L298N              PCNT                UART             │
│        │                  │                    │             │
│        └──────────────────┴────────────────────┘             │
│                           │                                  │
│                    FreeRTOS Tasks                            │
│                           │                                  │
│             ┌─────────────┴─────────────┐                    │
│             │                           │                    │
│       HTTP Control                 UDP Telemetry             │
│             │                           │                    │
└─────────────┼───────────────────────────┼────────────────────┘
              │                           │
           Browser                     Wi-Fi
                                          │
                                          ▼
┌──────────────────────── Host PC ─────────────────────────────┐
│                                                             │
│                     UDP Receiver                            │
│                          │                                  │
│                     Packet Parser                           │
│                     /           \                           │
│                RPM Data        LiDAR Batches                │
│                   │                 │                       │
│               Odometry        Scan Assembly                 │
│                   │                 │                       │
│                   └───────┬─────────┘                       │
│                           ▼                                 │
│                    Pose Synchronization                     │
│                           │                                 │
│                      Scan Matching                          │
│                           │                                 │
│                     Occupancy Grid                          │
│                           │                                 │
│                        Rerun                                │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

---

# Hardware

The robot currently uses:

- ESP32 development board
- RPLIDAR A1M8 2D LiDAR
- Four DC motors
- L298N motor driver
- Two wheel encoders
- Battery power system
- 5 V buck conversion for logic and sensors
- Wi-Fi connection to the host computer

The four motors are controlled as a differential-drive platform, with the left and right sides treated as two drive groups.

An IMU may be added later but is not currently required by the mapping pipeline.

---

# Firmware

The embedded software is written in **C using ESP-IDF and FreeRTOS**.

The ESP32 is responsible for:

- motor control
- wheel encoder acquisition
- RPM calculation
- RPLIDAR communication
- RPLIDAR packet parsing
- Wi-Fi networking
- browser-based remote control
- telemetry packet construction
- UDP transmission to the host

Heavy SLAM computation remains on the host PC.

---

## Firmware Architecture

A representative firmware structure is:

```text
embedded/
├── main/
│   └── main.c
│
├── components/
│   ├── motor/
│   │   ├── motor.c
│   │   └── motor.h
│   │
│   ├── encoder/
│   │   ├── encoder.c
│   │   └── encoder.h
│   │
│   ├── lidar/
│   │   ├── lidar.c
│   │   ├── lidar.h
│   │   ├── lidar_protocol.c
│   │   └── lidar_protocol.h
│   │
│   ├── telemetry/
│   │   ├── telemetry.c
│   │   └── telemetry.h
│   │
│   ├── wifi/
│   │   ├── wifi.c
│   │   └── wifi.h
│   │
│   └── web_control/
│       ├── web_control.c
│       ├── web_control.h
│       └── index.html
│
├── CMakeLists.txt
└── sdkconfig
```

The exact directory layout may vary as the project evolves.

---

# Motor Control

The ESP32 controls the four DC motors through an L298N motor driver.

The motors are grouped into left and right sides to create differential-drive motion.

Supported browser commands include:

```text
Forward
Backward
Left
Right
Stop
```

The embedded HTTP server exposes endpoints for these commands and forwards them to the motor-control component.

This allows the robot to be driven manually while sensor telemetry continues streaming independently to the host.

---

# HTTP Web Control

The ESP32 runs an HTTP server containing an embedded HTML control interface.

The control page is compiled into the firmware image and served directly by the ESP32.

Example control flow:

```text
Browser
   │
   │ HTTP POST
   ▼
ESP32 HTTP Server
   │
   ▼
Motor Command
   │
   ▼
L298N
   │
   ▼
Motors
```

Typical routes include:

```text
/forward
/backward
/left
/right
/stop
```

HTTP is used for robot control because command traffic is relatively infrequent and reliability is more important than minimum latency.

---

# Wheel Encoders

Wheel encoders provide feedback for differential-drive odometry.

The ESP32 uses its pulse counter hardware to count encoder pulses without requiring software to service every edge.

A periodic firmware task converts encoder counts into wheel RPM values.

The host receives:

```text
left_rpm
right_rpm
timestamp
sequence
```

These measurements are then converted into linear and angular robot motion.

---

# RPLIDAR Interface

The RPLIDAR A1 communicates with the ESP32 using UART.

Current configuration:

```text
Baud rate: 115200
Data bits: 8
Parity:    None
Stop bits: 1
```

The firmware starts the LiDAR motor and sends the standard RPLIDAR scan command.

Raw UART bytes are processed by a state-machine parser.

Each LiDAR measurement contains:

```text
angle
distance
quality
start flag
```

The `start_flag` marks the beginning of a new LiDAR revolution and is later used by the host to reconstruct complete 360° scans.

---

# FreeRTOS

Sensor acquisition and networking are separated into independent tasks.

Conceptually:

```text
LiDAR Task
    │
    ├── Read UART
    ├── Parse measurements
    └── Queue telemetry
             │
             ▼
       Telemetry Task
             │
             └── UDP → Host


Encoder Task
    │
    ├── Read PCNT
    ├── Calculate RPM
    └── Queue telemetry
             │
             ▼
       Telemetry Task
```

This prevents slow network operations from directly blocking sensor acquisition.

---

# UDP Telemetry

Sensor data is sent from the ESP32 to the host using UDP.

UDP was chosen because LiDAR generates continuous high-rate telemetry, where low latency is more useful than retransmitting old measurements.

The host listens on:

```text
0.0.0.0:5500
```

Two primary telemetry types are currently supported:

```text
0 = LiDAR
1 = Wheel RPM
```

---

## Common Telemetry Header

Each message begins with a common header containing:

```c
uint32_t type;
uint32_t sequence;
uint64_t timestamp_us;
```

Equivalent Python unpacking format:

```python
"<IIQ"
```

Fields:

| Field | Description |
|---|---|
| `type` | Telemetry message type |
| `sequence` | Packet sequence number |
| `timestamp_us` | ESP32 timestamp in microseconds |

Sequence numbers are used to detect missing and out-of-order UDP packets.

---

## RPM Message

RPM telemetry contains:

```c
float left_rpm;
float right_rpm;
```

Python representation:

```python
"<ff"
```

The resulting host-side object contains:

```text
sequence
timestamp_us
left_rpm
right_rpm
```

---

## LiDAR Messages

LiDAR samples are batched together before transmission to reduce UDP overhead.

Each sample contains:

```text
angle_deg
distance_mm
quality
start_flag
```

The host reads the packet's batch count and reconstructs each measurement.

Because UDP does not guarantee packet delivery, the receiver cannot assume every LiDAR sequence number will arrive.

---

# Host Software

The host portion is written in Python.

Current structure:

```text
host/
├── main.py
├── util.py
│
├── network/
│   ├── udp_receiver.py
│   └── packet_parser.py
│
├── telemetry/
│   └── messages.py
│
├── lidar/
│   └── scan.py
│
└── SLAM/
    ├── occupancy_grid.py
    └── odemetry.py
```

---

# UDP Receiver

The host creates a UDP socket and listens for incoming telemetry:

```python
socket(AF_INET, SOCK_DGRAM)
```

Unlike TCP, UDP does not use:

```text
listen()
accept()
```

Each datagram is received independently using:

```python
recvfrom()
```

The sender's address and binary payload are returned for each packet.

---

# Packet Parsing

Incoming packets are decoded based on their telemetry type.

```text
UDP packet
    │
    ▼
Common Header
    │
    ├── type = 0 ──→ LiDAR parser
    │
    └── type = 1 ──→ RPM parser
```

Binary parsing uses Python's `struct` module so that the host representation matches the C structures sent by the ESP32.

---

# LiDAR Scan Assembly

The RPLIDAR telemetry arrives as multiple UDP batches rather than one complete revolution.

The host stores batches in a dictionary indexed by sequence number:

```text
sequence → LiDAR batch
```

This allows packets to be reordered if UDP delivers them slightly out of order.

The `start_flag` contained in RPLIDAR samples determines actual scan boundaries.

Conceptually:

```text
start flag
    │
    ▼
samples...
samples...
samples...
    │
next start flag
    │
    ▼
complete revolution
```

Everything between two consecutive start flags becomes a `FullScan`.

If a required UDP sequence is missing, the incomplete scan is discarded instead of being inserted into the map with missing geometry.

---

# Wheel Odometry

The host estimates robot motion from the left and right wheel RPM values.

Wheel angular velocity is calculated as:

```text
ωwheel = RPM × 2π / 60
```

Linear wheel velocity is then:

```text
v = ωwheel × wheel_radius
```

For differential drive:

```text
linear velocity =
    (left_velocity + right_velocity) / 2
```

and angular velocity is derived from the difference between the two wheel velocities and the robot wheel base.

Current physical constants are approximately:

```text
Wheel radius: 0.03175 m
Wheel base:   0.13335 m
```

The pose maintained by odometry is:

```text
[x, y, theta]
```

where:

```text
x     = world X position
y     = world Y position
theta = robot heading
```

---

# Timestamp Synchronization

LiDAR and encoder measurements arrive independently.

Because a LiDAR scan may arrive between two odometry updates, the host stores a history of timestamped poses:

```text
timestamp → pose
```

For a scan timestamp between two odometry measurements:

```text
Pose A                 Pose B
t0 --------------------- t1
           ^
           |
      scan timestamp
```

the pose is interpolated.

X and Y use linear interpolation.

Heading interpolation uses the shortest angular difference so transitions around `-π` and `+π` do not produce incorrect rotations.

This gives each LiDAR scan an estimated robot pose corresponding to when that revolution began.

---

# Coordinate Frames

Three coordinate systems are used.

## LiDAR / Robot Frame

LiDAR points initially describe positions relative to the robot:

```text
"2 meters in front of the robot"
```

A sample is converted from polar form into Cartesian coordinates:

```text
x = distance × cos(angle)
y = distance × sin(angle)
```

---

## World Frame

The robot pose is used to rotate and translate local LiDAR points:

```text
world_x =
    robot_x +
    local_x cos(theta) -
    local_y sin(theta)

world_y =
    robot_y +
    local_x sin(theta) +
    local_y cos(theta)
```

This converts:

```text
LiDAR frame
    ↓
World frame
```

---

## Occupancy Grid Frame

World coordinates measured in meters are converted into grid cells.

```text
World coordinates
       ↓
world_to_grid_points()
       ↓
Grid indices
```

The current map is:

```text
30 m × 30 m
```

with:

```text
0.05 m / cell
```

giving a:

```text
600 × 600
```

cell occupancy grid.

---

# Occupancy Grid Mapping

The occupancy grid stores log-odds rather than directly storing probabilities.

The map begins as unknown:

```text
log odds = 0
probability = 0.5
```

Current updates are:

```text
Free-space update: -0.40
Occupied update:   +0.85
```

Values are limited to:

```text
-5 ≤ log odds ≤ +5
```

---

## Ray Tracing

For each LiDAR measurement, Bresenham's line algorithm traces the ray between:

```text
robot position
       ↓
LiDAR endpoint
```

Cells along the ray are marked as increasingly likely to be free.

```text
Robot
  │
  ├── free
  ├── free
  ├── free
  └── occupied endpoint
```

The final cell corresponding to the LiDAR return is marked as occupied.

---

# Scan Matching

Wheel odometry accumulates error because of:

- wheel slip
- encoder quantization
- unequal wheel behavior
- wheel-radius calibration errors
- wheel-base calibration errors

LiDAR scan matching is used to refine the odometry estimate.

The current implementation performs brute-force **correlative scan matching**.

Given an odometry pose:

```text
(x, y, theta)
```

the matcher searches nearby candidates.

Current search region:

```text
X:     ±0.15 m
Y:     ±0.15 m
Theta: ±5°
```

Current resolution:

```text
X:     0.05 m
Y:     0.05 m
Theta: 1°
```

For every candidate pose:

1. Transform the current LiDAR scan into world coordinates.
2. Convert each endpoint into an occupancy-grid cell.
3. Retrieve the occupancy probability at that location.
4. Average the endpoint scores.
5. Keep the candidate with the highest score.

Conceptually:

```text
Wheel odometry estimate
          │
          ▼
   Nearby candidate poses
          │
          ▼
Transform current scan
          │
          ▼
Compare endpoints to map
          │
          ▼
   Highest score wins
```

The first scan is inserted directly into the map because there is no existing map against which to perform scan matching.

---

# Current SLAM Pipeline

The current host pipeline is:

```text
ESP32
 │
 │ UDP
 ▼
Packet Receiver
 │
 ▼
Binary Parser
 │
 ├──────────── RPM
 │                │
 │                ▼
 │            Odometry
 │                │
 │                ▼
 │          Pose History
 │                │
 │                │
 └── LiDAR ───────┤
       │           │
       ▼           │
 Scan Assembly     │
       │           │
       └─────┬─────┘
             ▼
      Timestamp Matching
             │
             ▼
       Predicted Pose
             │
             ▼
        Scan Matching
             │
             ▼
       Corrected Pose
             │
             ▼
      Occupancy Mapping
             │
             ▼
          Rerun
```

---

# Visualization

The project uses **Rerun** to visualize:

- robot position
- transformed LiDAR points
- occupancy grid

This makes it possible to inspect the SLAM process while driving the robot.

The occupancy probabilities are converted into an 8-bit image before being sent to Rerun.

---

# Building the Firmware

The firmware uses ESP-IDF.

After installing and exporting the ESP-IDF environment:

```bash
cd embedded
idf.py build
```

Flash the ESP32:

```bash
idf.py -p /dev/ttyUSB0 flash
```

Monitor serial output:

```bash
idf.py -p /dev/ttyUSB0 monitor
```

Or combine them:

```bash
idf.py -p /dev/ttyUSB0 flash monitor
```

The exact serial device may differ between systems.

---

# Running the Host

The host currently requires Python with at least:

```text
numpy
rerun-sdk
```

Example installation:

```bash
pip install numpy rerun-sdk
```

Run the host:

```bash
cd host
python3 main.py
```

The UDP receiver listens on:

```text
UDP port 5500
```

The ESP32 must be configured to send telemetry to the IP address of the host machine.

---

# Network Architecture

HTTP control and UDP telemetry coexist over the same Wi-Fi connection.

```text
                    Wi-Fi
                      │
        ┌─────────────┴──────────────┐
        │                            │
        ▼                            ▼
     HTTP                         UDP
Browser → ESP32            ESP32 → Host PC
motor commands              sensor telemetry
```

HTTP handles relatively low-rate control traffic.

UDP handles continuous LiDAR and encoder telemetry.

---

# Known Limitations

### Wheel odometry drift

Wheel encoder odometry is inherently susceptible to accumulated error. Scan matching is being added to correct short-term drift.

### Scan matching is currently local

The matcher only searches a small region around the odometry prediction. Large odometry errors cannot currently be recovered.

### Pose correction is still being integrated

The current SLAM implementation is being updated so that scan-matching corrections persist between scans through a map-to-odometry transform.

### LiDAR motion distortion

A complete LiDAR revolution takes time to acquire.

Currently, all measurements in a revolution are transformed using a single robot pose. If the robot moves while the scan is being collected, this can distort walls and other geometry.

Future scan deskewing will compensate for robot motion during the revolution.

### UDP packet loss

UDP does not guarantee delivery.

The scan assembler therefore tracks sequence numbers and rejects incomplete scans when required LiDAR batches are missing.

---

# Technologies

### Embedded

- C
- ESP-IDF
- FreeRTOS
- GPIO
- UART
- PCNT
- Wi-Fi
- HTTP
- UDP
- Binary protocols
- RPLIDAR protocol parsing

### Host / Robotics

- Python
- NumPy
- UDP sockets
- Binary serialization
- Differential-drive odometry
- Sensor timestamp synchronization
- LiDAR coordinate transformations
- Bresenham ray tracing
- Log-odds occupancy mapping
- Correlative scan matching
- Rerun visualization

---

## License

This project is currently intended as a personal robotics and embedded-systems project.