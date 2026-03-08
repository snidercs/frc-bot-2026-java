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

## Power-Up Initialization

When the robot powers on, each subsystem automatically configures its hardware and establishes known starting positions. No manual intervention is needed for this sequence — it runs entirely in the `RobotContainer` and subsystem constructors.

### Startup sequence

1. **AdvantageKit logger** — `Robot` starts the AdvantageKit logger before anything else. On a real robot it writes to a USB stick and publishes to NetworkTables; in replay mode it reads from a prior log file.
2. **Brownout threshold** — `robotInit()` sets the roboRIO brownout voltage to 6.0 V to prevent nuisance brownouts under heavy CAN load.
3. **Drivetrain (swerve)** — `CommandSwerveDrivetrain` initializes all four swerve modules (Kraken X60 drive + steer, CANcoder) at a 250 Hz odometry update rate. Operator perspective is automatically set for the current alliance color once the Driver Station connects.
4. **PathPlanner** — `configurePathPlanner()` loads the robot configuration from PathPlanner GUI settings and registers the `AutoBuilder` so autonomous routines can be selected. All named commands (shooter, intake, aiming) are registered at this time.
5. **Turret** — The three turret motors are configured with current limits, PID gains, and software limits:
   - **Rotation motor** — set to brake mode, position zeroed to 0.25 rotations (90°, facing right of robot). Software limits constrain travel to −0.055 to +0.25 rotations.
   - **Shooter flywheel** — configured for velocity control, coast mode.
   - **Uptake feeder** — configured for velocity control, coast mode.
6. **Climber** — The Kraken X60 is configured with a 180:1 gear ratio, brake mode, and software-limited travel (0 to −3.109 rotations). The encoder position is zeroed to 0.0 on startup, defining the current position as the forward (retracted) limit.
7. **Intake** — Both top and bottom Kraken X44 motors are configured with current limits. The bottom motor is inverted so both rollers spin inward together. Both run in coast mode.
8. **Vision** — `VisionMulti` loads the `k2026RebuiltAndymark` AprilTag field layout and creates a `PhotonPoseEstimator` for each of the four cameras (FL, FR, BL, BR). Cameras begin streaming immediately.
9. **Controller bindings** — Joystick or gamepad bindings are wired based on `Config.GAMEPAD`. A disabled-mode idle command keeps the swerve modules in an idle state until a match begins.

### What "zeroing" means

The climber and turret rotation motors use relative encoders (built into the Kraken/TalonFX). On power-up, the code calls `setPosition()` to seed a known value:

| Motor | Startup position | Assumption |
|---|---|---|
| Climber | 0.0 rotations (fully retracted) | Robot was powered on with the climber fully retracted |
| Turret rotation | 0.25 rotations (90° right) | Robot was powered on with the turret facing its mechanical home |

If these assumptions are wrong (e.g., the turret was bumped during transport), the software limits will be offset. Use the turret zero button (stick 1 button 3) to re-zero after manually centering the turret.

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

## Robot Localization (How the Robot Knows Its Position)

The robot continuously estimates its field-relative position (X, Y, heading) by fusing two independent data sources: **wheel odometry** and **AprilTag vision**.

### Wheel odometry

The swerve drivetrain runs a high-frequency odometry loop at **250 Hz** ([`RobotContainer.java`](src/main/java/frc/robot/RobotContainer.java#L68)). Each cycle, the `CommandSwerveDrivetrain` reads the four drive-motor encoders and four CANcoders to compute incremental movement. Because this runs on-device in the CTRE Phoenix 6 stack, it captures fine-grained motion even between the main robot loop ticks.

Odometry alone drifts over time — wheel slip, carpet texture changes, and minor gear backlash accumulate error. Vision corrections compensate for this drift.

### AprilTag vision

Four PhotonVision cameras (Front-Left, Front-Right, Back-Left, Back-Right) are mounted at fixed positions on the chassis. Each camera's 3D transform relative to robot center is defined in [`Vision.java`](src/main/java/frc/robot/Vision.java#L82-L106) (`ROBOT_TO_CAMERA[]`).

The [`VisionMulti`](src/main/java/frc/robot/VisionMulti.java) class implements `Vision.VisionIO` and manages all four cameras:

1. **Capture** — Each camera is polled every robot cycle via `PhotonCamera.getAllUnreadResults()` ([`VisionMulti.java`](src/main/java/frc/robot/VisionMulti.java#L93)).
2. **Gate** — Results are filtered to reject measurements that are stale (latency > 0.5 s), ambiguous (ambiguity > 0.3), or produce poses outside the field boundaries (16.46 m × 8.21 m) ([`VisionMulti.java`](src/main/java/frc/robot/VisionMulti.java#L97-L126)).
3. **Estimate** — Accepted results are passed to a `PhotonPoseEstimator`, which uses the known AprilTag field layout (`k2026RebuiltAndymark`) and the camera mounting transform to compute a robot pose on the field ([`VisionMulti.java`](src/main/java/frc/robot/VisionMulti.java#L111-L115)).
4. **Standard deviations** — Each measurement is tagged with distance-scaled uncertainty values (X/Y: 1 cm base + 5 cm per meter; heading: 0.01 rad base + 0.02 rad per meter) so farther tags carry less weight ([`VisionMulti.java`](src/main/java/frc/robot/VisionMulti.java#L146-L150)).

### Sensor fusion

The odometry and vision streams are fused in [`Robot.robotPeriodic()`](src/main/java/frc/robot/Robot.java#L77-L86). Every cycle, vision measurements are fed to `drivetrain.addVisionMeasurement()` along with their standard deviations. The underlying WPILib pose estimator uses a Kalman-filter-style update: odometry provides the prediction step, and each vision measurement applies a correction weighted by its uncertainty. The result is a single best-estimate `Pose2d` available via `drivetrain.getState().Pose`.

### Field-centric heading

The driver can reset the field-centric forward direction by pressing the seed button (stick 0 button 8, or left bumper on gamepad), which calls `drivetrain.seedFieldCentric()` ([`RobotContainer.java`](src/main/java/frc/robot/RobotContainer.java#L211)). This re-zeros the gyro heading so "forward" on the stick matches the robot's current facing. The operator perspective is also automatically adjusted for alliance color in [`CommandSwerveDrivetrain.periodic()`](src/main/java/frc/robot/subsystems/CommandSwerveDrivetrain.java#L194-L207).

### Telemetry

The fused robot pose and swerve module states are published to NetworkTables by the [`Telemetry`](src/main/java/frc/robot/Telemetry.java) class, which populates the `DriveState/Pose`, `DriveState/Speeds`, and `Pose/robotPose` topics. Individual camera poses and rejection counts are also published under `Vision/` keys by `VisionMulti`.

### Data flow summary

```
┌──────────────────────┐     ┌──────────────────────────┐
│  Swerve Module       │     │  PhotonVision Cameras    │
│  Encoders + CANcoders│     │  (FL, FR, BL, BR)        │
│  250 Hz              │     │  ~30 fps each            │
└─────────┬────────────┘     └────────────┬─────────────┘
          │ (a)                           │ (b)
          ▼                               ▼
┌──────────────────────┐     ┌──────────────────────────┐
│  Wheel Odometry      │     │  VisionMulti             │
│  (prediction step)   │     │  (gate → estimate → σ)   │
└─────────┬────────────┘     └────────────┬─────────────┘
          │ (c)                           │ (d)
          └───────────┬───────────────────┘
                      ▼
          ┌───────────────────────┐
          │  WPILib Pose Estimator│
          │  (Kalman filter)      │
          │  → Pose2d             │
          └───────────┬───────────┘
                      │
          ┌───────────┴───────────┐
          │ (e)                   │ (f)
          ▼                       ▼
┌──────────────────┐   ┌──────────────────┐
│  Autonomous      │   │  Turret Aiming   │
│  (PathPlanner)   │   │  (hub/tower)     │
└──────────────────┘   └──────────────────┘
```

Each arrow is a method call — nothing is pushed asynchronously. Downstream consumers **pull** the latest fused pose from `drivetrain.getState().Pose` whenever they need it.

| Arrow | What happens | Where in code |
|---|---|---|
| **(a)** Encoders → Odometry | CTRE's `SwerveDrivetrain` base class reads drive-motor encoders + CANcoders at 250 Hz internally | built into Phoenix 6 `SwerveDrivetrain` |
| **(b)** Cameras → VisionMulti | `PhotonCamera.getAllUnreadResults()` polls each camera for new frames | [`VisionMulti.getMeasurements()`](src/main/java/frc/robot/VisionMulti.java#L93) |
| **(c)** Odometry → Pose Estimator | Automatic — odometry is the prediction step inside the `SwerveDrivetrain` pose estimator | built into Phoenix 6 `SwerveDrivetrain` |
| **(d)** VisionMulti → Pose Estimator | `drivetrain.addVisionMeasurement(pose, timestamp, stdDevs)` feeds each accepted measurement into the Kalman filter | [`Robot.robotPeriodic()`](src/main/java/frc/robot/Robot.java#L80) |
| **(e)** Pose Estimator → PathPlanner | A lambda `() -> getState().Pose` is registered as PathPlanner's pose supplier during init | [`configurePathPlanner()`](src/main/java/frc/robot/subsystems/CommandSwerveDrivetrain.java#L220) |
| **(f)** Pose Estimator → Turret Aiming | Aim commands read `drivetrain.getState().Pose` and compute the angle/distance to `Vision.hubPosition()` | [`Robot.robotPeriodic()`](src/main/java/frc/robot/Robot.java#L88-L90) |

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
