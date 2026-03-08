# frc-bot-2026-java

Java robot code for **Team 9431 – Indy**, competing in the 2026 FRC season.

This project is a complete port of the original
[C++ codebase](https://github.com/snidercs/frc-bot-2026-cxx) to Java, targeting
the same hardware and vendor libraries.

## Tech Stack

| Component | Version |
|---|---|
| WPILib / GradleRIO | 2026.2.1 |
| CTRE Phoenix 6 | 26.1.0 |
| PathPlanner | 2026.1.2 |
| PhotonVision | 2026.2.1 |
| Java | 17 |

## Subsystems

| Subsystem | Description |
|---|---|
| **CommandSwerveDrivetrain** | Phoenix 6 swerve drive with SysId characterization and PathPlanner autonomous |
| **Intake** | Dual Kraken X44 motors, voltage-controlled intake/eject/stutter |
| **Climber** | Single Kraken X60 with software-limited travel and 180:1 gear ratio |
| **Turret** | 3-motor turret — rotation (position PID), shooter flywheel (velocity), uptake feeder (velocity) |
| **VisionMulti** | 4-camera PhotonVision AprilTag pose estimation with ambiguity/latency/bounds gating |
| **Telemetry** | Swerve state publishing to NetworkTables + Mechanism2d visualization |

## Controls

The robot supports two controller modes selected at compile time via `Config.GAMEPAD`:

- **Dual flight sticks** (default) — stick 0 for translation + intake/climber, stick 1 for rotation + shooter
- **Xbox gamepad** — left stick translation, right stick rotation, bumpers/triggers for mechanisms

## Building

```bash
./gradlew build
```

## Deploying

```bash
./gradlew deploy
```

Requires the roboRIO to be reachable on the network (USB, ethernet, or radio).

## Project Structure

```
src/
├── main/
│   ├── java/frc/robot/
│   │   ├── Config.java                 # Static robot configuration constants
│   │   ├── InputUtil.java              # Joystick deadband + exponential curve shaping
│   │   ├── Main.java                   # Entry point
│   │   ├── Robot.java                  # TimedRobot lifecycle
│   │   ├── RobotContainer.java         # Subsystem wiring + controller bindings
│   │   ├── Telemetry.java              # Swerve telemetry
│   │   ├── Vision.java                 # Vision constants, interfaces, records
│   │   ├── VisionMulti.java            # Multi-camera PhotonVision implementation
│   │   ├── generated/
│   │   │   └── TunerConstants.java     # Tuner X swerve module constants
│   │   └── subsystems/
│   │       ├── CommandSwerveDrivetrain.java
│   │       ├── Intake.java
│   │       ├── Climber.java
│   │       └── Turret.java
│   └── deploy/pathplanner/             # PathPlanner paths and auto routines
└── test/java/frc/robot/
    └── InputUtilTest.java              # JUnit 5 tests for input shaping
```

## Contributors

The original C++ codebase was written by:

- **Michael Fisher** ([@mfisher31](https://github.com/mfisher31))
- **Kawi Minn** ([@kawiminn](https://github.com/kawiminn))

## Porting Notes

This project was ported from the C++ version. For a detailed breakdown of
architecture changes, API differences, and gotchas encountered during the port,
see [porting-from-cxx.md](porting-from-cxx.md).

The original C++ source is at
[snidercs/frc-bot-2026-cxx](https://github.com/snidercs/frc-bot-2026-cxx).
