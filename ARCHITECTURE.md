# Architecture

CleanBot splits work between the Raspberry Pi 5 (sensing, detection, control) and a host PC (SLAM, visualisation). Every link is UDP, chosen for low latency; a dropped packet is simply superseded by the next one.

## Processes

| Process | Runs on | Role |
|---|---|---|
| `rpi5_sender.py` | Pi | Captures camera + IMU, runs YOLO, streams to PC, forwards pose/detections to the motor brain |
| `rpi5_motor_brain_slam.py` | Pi | Behaviour state machine, zig-zag explorer, A* planner, PID, BTS7960 PWM |
| `pc_receiver.py` | PC | Bridges Pi ⇄ SLAM, draws overlays, hosts the Open3D viewer (separate process) |
| `slam_receiver` | PC | ORB-SLAM3 monocular-inertial tracker; returns pose and map points |

Start order: `slam_receiver` → `pc_receiver.py` → `rpi5_sender.py` → `rpi5_motor_brain_slam.py`.

## Port map

| Port | From → To | Payload | Rate |
|---|---|---|---|
| 5000 | Pi sender → PC receiver | JPEG camera frame | 25 Hz |
| 5001 | Pi sender → PC receiver | IMU packet (`struct 'd6f'`: timestamp, ax, ay, az [m/s²], gx, gy, gz [rad/s]) | 100 Hz |
| 5002 | Pi sender → PC receiver | Detections, JSON list | ~10 Hz |
| 5003 | PC receiver → Pi sender | Pose text `"x y z state"` | per SLAM frame |
| 6100 | Pi sender → motor brain (localhost) | Pose text, forwarded | per SLAM frame |
| 6101 | Pi sender → motor brain (localhost) | Detections JSON, forwarded | ~10 Hz |
| 9997 | PC receiver → slam_receiver | JPEG frame | 25 Hz |
| 9998 | PC receiver → slam_receiver | IMU binary packet | 100 Hz |
| 9999 | slam_receiver → PC receiver | Pose text | per frame |
| 9996 | slam_receiver → PC receiver | Map points, chunked (200 points / packet, 1-byte header) | periodic |

## SLAM state codes

| Code | Meaning | Robot behaviour |
|---|---|---|
| 0 | `TRACKING` | Explore / go to dirt |
| 1 | `RECENTLY_LOST` | Spin to recover |
| 2 | `LOST` | Spin to recover |
| 3 | `INITIALIZING` | Spin to find features |

## Navigation

- **Grid:** occupancy grid with 5 cm cells; liquid obstacles above 0.5 confidence are written into it.
- **Planner:** 8-connected A* (diagonal cost 1.414) with a Manhattan heuristic. If no path is found the robot drives directly at the target.
- **Target selection:** in `SMALL_DIRT_MODE`, only boxes between 400 and 12 000 px² are considered, closest first. Larger boxes usually mean the robot is already on top of the dirt.
- **Arrival:** within 12 cm of the target → wait 1 s → resume exploring.
- **Exploration:** zig-zag sweep of 8 legs, 1.5 m long, 0.40 m apart, at 0.15 m/s (slow for better SLAM features).
- **Control:** PID on heading and speed, converted to wheel duty cycles by differential-drive kinematics (0.20 m wheel base, 0.25 m/s max, 60 % max duty).
