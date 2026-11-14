# Architecture

## Product
The user moves a light dummy pestle in a sensed dummy mortar: the **Leader**.
A robot arm (SO-101), the **Follower**, grinds with a real pestle.

## Domain terms
- **Leader**: the sensed dummy pestle and mortar. It emits Leader Samples.
- **Follower**: the robot that grinds. It is simulated or physical.
- **Leader Sample**: one calibrated reading in SI units: timestamp, orientation, pressure, tip position, contact.
- **Grind Intent**: the user's intent over a 1–2 stroke window: pattern, rate, radius, direction, requested pressure.
- **Follower Command**: the target the Follower executes for one tick.
- **Mode**: the strategy that maps Leader Samples to Follower Commands.
- **Safety Envelope**: the limits on force, speed and workspace, plus the stale-data timeout.

## Data flow

```
Leader source --> interpreter --> safety governor --> Follower
 Leader Sample    Grind Intent    safe Follower        FollowerState
   (100 Hz)        (10 Hz)        Command
                       \______________|______________/
                                   recorder
```

The host is one process with one synchronous tick loop and an injected clock.

## Modules
Each module has a small interface and hides the detail listed.

| Module | Interface | Hides |
|---|---|---|
| `protocol` | Encode and decode messages | nanopb schema, COBS, CRC16, versioning |
| `leader-firmware` (C++) | Emit calibrated Leader Samples | Drivers, IMU fusion, load-cell tare and scale, center of pressure, calibration storage |
| `interpreter` | Leader Sample stream → Grind Intent → Follower Command | Stroke detection, tremor rejection, Mode |
| `safety governor` | Follower Command → safe Follower Command | Safety Envelope limits, stale-data watchdog |
| `follower` | Execute a Follower Command | Simulator or hardware specifics |
| `recorder` | Write and read session logs | Log format |

## Seams
Each seam has one interface and at least two adapters.

| Seam | Adapters |
|---|---|
| Leader source | replay, synthetic, simulated (PyBullet); later serial |
| Follower | PyBullet; later MuJoCo, Gazebo, physical SO-101 |
| Mode | Mirror, Interpret |

## Modes
- **Mirror**: the Follower Command is the Leader's tip position and pressure, low-pass filtered
  at 5 Hz to remove tremor (4-12 Hz). Any motion works. The delay is about 70 ms.
  A Leader Sample has a position only during contact. When contact starts at a new point,
  the Follower travels there lifted and presses after it arrives.
- **Interpret**: the Follower estimates a Grind Intent over a 2 s window and executes an ideal
  stroke. It knows circular strokes and pounding at one point. Other motion gives idle.
- Both Modes report the estimated Grind Intent for display and recording.

## Simulation
- The simulated Leader and the simulated Follower share one PyBullet world.
- Each has a mortar and a pestle. Each pestle is a free rigid body.
- The Leader pestle is a light dummy (30 g). The Follower pestle is a stone pestle (400 g).
- Leader: a force-capped constraint holds the pestle. The cap sets how hard the hand presses.
- Follower: the gripper jaws close on the pestle. The grasp is rigid sideways. The pestle
  slides a few millimetres along its axis, and the adapter sets the force along that axis.
  The arm carries the pestle's weight and feels the pressing force as a reaction.
- The simulated Leader reads force and centre of pressure from contact, as a load-cell plate does.
- The Follower arm uses joint-space position control with capped torque.
- A damped least-squares solver computes the joint targets. It does not use PyBullet's IK.

## Protocol
- Payload: nanopb messages.
- Frame: version, sequence number, message type, payload, CRC16. Each frame is COBS-encoded.
- Transport: USB serial.
- C++ and Python share golden vectors.
- Calibration runs on the firmware. The wire carries calibrated SI units.

## Rates
- Leader: 100 Hz.
- Grind Intent: 10 Hz.
- Follower: simulator step rate.

## Fault rule
After about 100 ms without a valid Leader Sample, the Follower releases pressure, lifts a few millimetres and holds.

## Leader hardware
- ESP32 with PlatformIO and the Arduino framework.
- A BNO055-class IMU in the dummy pestle.
- A load-cell plate (4 cells, HX711) under the dummy mortar. It gives pressure and tip position.
- Hardware-free logic lives in `firmware/lib/core/`. Native tests cover it.

## Milestones
1. Repo skeleton, `make check`, CI.
2. `protocol`: schema, framing, golden vectors in both languages.
3. `interpreter` (Interpret Mode) and `safety governor`.
4. PyBullet SO-101 Follower and replay Leader, headless end-to-end test.
5. Leader hardware: firmware, calibration, serial adapter.
6. Demo: real Leader drives the simulated SO-101.
7. MuJoCo adapter.
8. Mirror Mode (done early, with Milestone 4).
9. ROS2 bridge and Gazebo.
10. Physical SO-101 adapter.
