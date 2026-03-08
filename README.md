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

See [CONTROLS.md](CONTROLS.md) for the complete button/axis mapping, turret
aiming modes, autonomous named commands, and configuration reference.

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

## Field Setup & Calibration

Follow these steps every time the robot arrives at a new field or match venue.

### 1. Power on & connect

1. Power on the robot and wait for the roboRIO to boot (status LED solid green).
2. Connect the Driver Station laptop to the robot via USB, ethernet, or the FMS radio.
3. Verify all four cameras (FL, FR, BL, BR) show a live feed in the PhotonVision dashboard at `http://photonvision.local:5800`.

### 2. Reset field-centric heading

The swerve drivetrain uses a field-centric reference frame. **Before each match**, align the robot facing directly away from your driver station, then press:

- **Dual sticks:** button 8 on stick 0
- **Gamepad:** left bumper

This calls `seedFieldCentric()` and sets the current heading as the field-forward direction. Repeat any time the heading drifts or after the robot is physically repositioned.

### 3. Verify vision cameras

1. Open the PhotonVision web UI and confirm each camera detects AprilTags on the field.
2. The robot code uses the `k2026RebuiltAndymark` AprilTag field layout — no manual tag ID entry is needed.
3. Watch the Driver Station console for vision rejection warnings. High rejection counts (visible via `SmartDashboard` under the Vision keys) may indicate a camera is misaligned or obstructed.

### 4. Check turret zero

The turret rotation motor uses a position PID. If the turret was bumped during transport:

1. Manually center the turret so it faces straight forward.
2. Press **button 3 on stick 1** (dual sticks) to zero the rotation encoder.

### 5. Select auto routine

1. Open `SmartDashboard` or `Shuffleboard`.
2. Use the **AutoChooser** widget to select the desired autonomous routine. The default is `Backup-Shoot-Left`.
3. Available autos are listed in [CONTROLS.md](CONTROLS.md).

### 6. Pre-match checklist

| Check | What to look for |
|---|---|
| Battery voltage | ≥ 12.4 V on Driver Station |
| Brownout threshold | Set to 6.0 V (automatic in code) |
| Joystick mapping | Driver Station shows correct controllers in slots 0 and 1 (or slot 0 for gamepad) |
| Alliance color | Confirmed in Driver Station — turret aiming uses `hubPosition()` / `towerPosition()` which are alliance-aware |
| Cameras connected | All four cameras streaming in PhotonVision dashboard |
| Intake clear | No game pieces jammed in intake or uptake |
| Climber retracted | Climber at bottom limit before match start |

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
