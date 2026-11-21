# Hardware & Mechanical Architecture

This document summarizes the mechanical design, bill of materials, joint kinematics, and electrical architecture for **Nomios**.

## Bill of Materials (~£700 Target)

| Subsystem | Component | Qty | Notes | Approx. Cost |
| :--- | :--- | :---: | :--- | ---: |
| Compute | Arduino Uno Q 4GB | 1 | Main onboard Linux ARM64 controller | ~£55 |
| Actuators | Feetech STS3215 serial bus servos | 10 | 7.4 V, 2.94 N·m stall, 12-bit magnetic encoders | ~£190 |
| Servo interface | TTL serial driver / UART adapter | 1 | 1 Mbps half-duplex daisy-chain bus | ~£10 |
| Vision | Raspberry Pi Camera v2 (8 MP) | 3 | 2x front stereo (80 mm baseline) + 1x top head | ~£75 |
| Orientation | BNO085 9-DoF IMU breakout | 1 | Gravity vector, gyro, and accel over I2C | ~£20 |
| Audio | Compact USB microphone | 1 | Bioacoustic bird/wildlife call capture | ~£15 |
| Power | 2S LiPo packs (7.4 V) + 5 V / 5 A UBEC | 2 + 1 | Direct servo rail + regulated logic supply | ~£60 |
| Structure | PETG filament (~1 kg) | 1 | Self-printed torso shell, brackets, and links | ~£25 |
| Fasteners & bearings | M3 heat-set inserts, M3 screws, F623ZZ bearings | Set | Joint pivots, servo horns, and housing fasteners | ~£40 |
| Wiring & misc | Silicone wire, XT30/JST connectors, switches, spares | — | Power distribution, fuses, spare servos/prints | ~£210 |
| **Total** | | | | **~£700** |

*(Prices reflect typical student/UK hobbyist sourcing and include a hardware spare allowance.)*

## Mechanical Architecture & Kinematics

Each leg uses a symmetric **5-DoF** serial chain (`hip_yaw -> hip_roll -> hip_pitch -> knee_pitch -> ankle_pitch`), giving 10 actuated joints in total. To minimize cantilevered play on printed parts, servo clamshell housings remain bolted to the parent link while custom printed U-brackets (`parts/sts3215_housing/u_bracket_v2.SLDPRT`) rotate with the child link around the servo output shaft.

| Link / Joint | Parent Link | Joint Range (deg) | Modeled Link Mass (kg) | Notes |
| :--- | :--- | :---: | :---: | :--- |
| `torso_base` | `world` (freejoint) | — | 0.4800 | Head shell, Uno Q + housing, media carrier, 2x battery packs, 3x cameras, hip-yaw stators |
| `joint_<side>_hip_yaw` | `torso_base` | -45 to +45 | 0.0840 | Vertical-ish yaw rotation from pelvis |
| `joint_<side>_hip_roll` | `<side>_hip_yaw_link` | -45 to +45 | 0.0754 | Lateral leg abduction/adduction |
| `joint_<side>_hip_pitch` | `<side>_hip_roll_link` | -90 to +90 | 0.0960 | Upper shin assembly and knee U-bracket |
| `joint_<side>_knee_pitch` | `<side>_thigh_link` | -120 to +120 | 0.0861 | Lower shin assembly and ankle U-bracket |
| `joint_<side>_ankle_pitch` | `<side>_shin_link` | -70 to +70 | 0.1224 | Ankle actuator and rigid foot sole |

- **Standing envelope (neutral CAD pose):** ~0.634 m height, ~0.325 m width across hips/shoulders, ~0.092 m front-to-back depth.
- **Total modeled mass:** 1.408 kg (`0.4800 + 2 * 0.4639 kg`).
- **Actuator modeling:** In `models/system.xml`, each servo is modeled as a position-controlled `<position>` actuator (`kp="45"`, `kv="2.5"`, `forcerange="-3.0 3.0"` N·m) rather than an ideal torque `<motor>`, matching the STS3215's internal position loop.

### Vision & Sensor Placement

- **Front stereo pair (`left_stereo_cam`, `right_stereo_cam`):** Mounted inside `pi_camera_v2_stereo_bracket_80mm.SLDPRT` on the front face of the torso with an 80 mm horizontal baseline (`y = ±0.040 m`, `fovy = 48.8°`).
- **Top head camera (`top_head_cam`):** Mounted above the stereo pair with a slight upward tilt for canopy/tall flora framing.
- **IMU (`imu_site`):** Placed at the center of the `Arduino_UNO_Q` PCB inside the torso cavity.
- **Foot contact sites (`left_foot_site`, `right_foot_site`):** Placed flush on the bottom sole pads of `foot_v1` for ground reaction force and touch sensing in simulation.

## Power & Wiring Architecture

Logic power and actuator power are isolated on separate rails so high servo stall currents do not brown out the onboard computer or cameras:

```text
  [ 2x 2S LiPo (7.4 V) ]
           |
           +------------------------------+
           | (Direct high-current rail)   | (5 V / 5 A UBEC regulator)
           v                              v
  [ 10x Feetech STS3215 ]        [ Arduino Uno Q 4GB ]
    (1 Mbps TTL daisy-chain)       |--- UART / TTL ---> Servo Bus (Pos + Telemetry)
                                   |--- I2C ----------> BNO085 9-DoF IMU
                                   |--- Camera I/F ---> 3x Pi Camera v2 (8 MP)
                                   +--- USB ----------> Microphone (Bioacoustics)
```

## CAD Source Organization (`parts/`)

The SolidWorks files in `parts/` preserve their original assembly filenames so SolidWorks mate and subassembly references remain intact:

- `parts/full assembly.SLDASM` — Top-level robot assembly.
- `parts/sts3215_housing/` — Clamshell stator mounts (`clamshell_base`, `clamshell_cap`), U-brackets (`u_bracket`, `u_bracket_v2`), and STS3215 servo STEP/SolidWorks models.
- `parts/v1/` — Foot, lower shin, and upper shin structural parts.
- `parts/Arduino_UNO_Q.SLDPRT`, `parts/arduino_uno_q_housing.SLDPRT`, `parts/media carrier.SLDPRT`, `parts/pi_camera_v2_stereo_bracket_80mm.SLDPRT`, `parts/head.SLDPRT` — Torso electronics enclosure and sensor mounts.

`scripts/build_model.py` reads the SolidWorks G3 binary streams (`swXmlContents/COMPINSTANCETREE` and `Contents/DisplayLists`) directly in Python, allowing `meshes/`, `urdf/nomios.urdf`, `models/system.xml`, and `models/scene.xml` to be rebuilt on any OS without a SolidWorks license.
