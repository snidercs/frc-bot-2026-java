# Porting frc-bot-2026-cxx to frc-bot-2026-java

This document describes the complete port of Team 9431's 2026 FRC robot code ("Indy")
from C++ to Java. Both projects target **WPILib GradleRIO 2026.2.1** with identical
vendor libraries and hardware.

---

## Table of Contents

1. [Overview](#overview)
2. [Project Structure](#project-structure)
3. [Dependency & Build System](#dependency--build-system)
4. [File Mapping](#file-mapping)
5. [Architecture Changes](#architecture-changes)
6. [Subsystem-by-Subsystem Notes](#subsystem-by-subsystem-notes)
7. [API Differences & Gotchas](#api-differences--gotchas)
8. [What Was Dropped](#what-was-dropped)
9. [Testing](#testing)
10. [Build Verification](#build-verification)

---

## Overview

| Property | C++ | Java |
|---|---|---|
| Language | C++20 | Java 17 |
| Build plugin | `edu.wpi.first.GradleRIO` 2026.2.1 (cpp) | `edu.wpi.first.GradleRIO` 2026.2.1 (java) |
| Test framework | GoogleTest | JUnit 5 |
| Entry point | `int main()` (WPILib macro) | `frc.robot.Main` → `RobotBase.startRobot(Robot::new)` |
| Config system | Lua (`sol2` + `robot/config.lua`) | Static Java constants (`Config.java`) |
| Robot name | Indy | Indy |
| Team number | 9431 | 9431 |

### Vendor Libraries (identical)

| Library | Version | Vendordep JSON |
|---|---|---|
| CTRE Phoenix 6 | 26.1.0 | `Phoenix6-26.1.0.json` |
| PathPlanner | 2026.1.2 | `PathplannerLib.json` |
| PhotonVision | 2026.2.1 | `photonlib.json` |
| WPILib New Commands | (bundled) | `WPILibNewCommands.json` |

The C++ project also carried `LuaBot.json` for the Lua scripting runtime — this was
**not** ported (see [What Was Dropped](#what-was-dropped)).

---

## Project Structure

### C++ layout

```
frc-bot-2026-cxx/
├── build.gradle            # cpp plugin, GoogleTest
├── robot/
│   └── config.lua          # Runtime config (deployed to roboRIO)
├── src/
│   ├── *.hpp / *.cpp       # Robot source (flat directory)
│   ├── generated/          # Tuner X swerve constants
│   ├── luabot/             # Lua scripting glue
│   ├── sol/                # sol2 Lua binding headers
│   └── main/deploy/pathplanner/  # PathPlanner paths & autos
├── test/cpp/               # GoogleTest sources
└── vendordeps/             # Vendor JSON files
```

### Java layout

```
frc-bot-2026-java/
├── build.gradle            # java plugin, JUnit 5
├── src/
│   ├── main/
│   │   ├── java/frc/robot/
│   │   │   ├── Config.java
│   │   │   ├── InputUtil.java
│   │   │   ├── Main.java
│   │   │   ├── Robot.java
│   │   │   ├── RobotContainer.java
│   │   │   ├── Telemetry.java
│   │   │   ├── Vision.java
│   │   │   ├── VisionMulti.java
│   │   │   ├── generated/
│   │   │   │   └── TunerConstants.java
│   │   │   └── subsystems/
│   │   │       ├── CommandSwerveDrivetrain.java
│   │   │       ├── Intake.java
│   │   │       ├── Climber.java
│   │   │       └── Turret.java
│   │   └── deploy/pathplanner/   # Copied as-is from C++
│   └── test/java/frc/robot/
│       └── InputUtilTest.java
└── vendordeps/             # Copied as-is from C++ (minus LuaBot.json)
```

---

## Dependency & Build System

### build.gradle changes

| Aspect | C++ | Java |
|---|---|---|
| Plugin | `id "cpp"` + `id "google-test-test-suite"` | `id "java"` |
| Source model | `model { components { frcUserProgram(NativeExecutableSpec) ... } }` | Standard `sourceSets` (implicit) |
| Artifact | `FRCNativeArtifact` | `FRCJavaArtifact` |
| Fat jar | N/A | `jar { from configurations.runtimeClasspath ... }` |
| Testing | `GoogleTestTestSuiteSpec` | JUnit Jupiter 5.10.1 |
| Deploy (static) | Two `FileTreeArtifact` blocks (robot/ + pathplanner/) | Single `FileTreeArtifact` from `src/main/deploy` |
| Lua modules | Custom `extractLuaModules` task | Removed |
| SSH deploy | `sshAntTask` configuration | Not needed |
| Compiler opts | N/A | `-XDstringConcat=inline` |

### settings.gradle

Identical between both projects — standard GradleRIO plugin management pointing at
`~/wpilib/2025/maven`.

### Infrastructure files copied unchanged

- `gradle/wrapper/gradle-wrapper.properties` + `gradle-wrapper.jar`
- `gradlew` / `gradlew.bat`
- `simgui-ds.json`
- `tuner-project.json`
- `vendordeps/` (all except `LuaBot.json`)
- `src/main/deploy/pathplanner/` (all paths, autos, navgrid, settings)

---

## File Mapping

| C++ Source | Java Target | Notes |
|---|---|---|
| `robot/config.lua` + `config.hpp/cpp` + `scripting.hpp/cpp` | `Config.java` | Lua runtime → static constants |
| `inpututil.hpp` | `InputUtil.java` | Header-only → static utility class |
| `vision.hpp` | `Vision.java` | Constants, records, interfaces |
| `visionmulti.hpp/cpp` | `VisionMulti.java` | Multi-camera PhotonVision |
| `drivetrain.hpp/cpp` | `subsystems/CommandSwerveDrivetrain.java` | CTRE swerve drivetrain |
| `telemetry.hpp/cpp` | `Telemetry.java` | NT publishing + Mechanism2d |
| `intake.hpp/cpp` | `subsystems/Intake.java` | Dual-motor intake |
| `climber.hpp/cpp` | `subsystems/Climber.java` | Single-motor climber |
| `turret.hpp/cpp` | `subsystems/Turret.java` | 3-motor turret/shooter |
| `container.hpp/cpp` | `RobotContainer.java` | Bindings + auto setup |
| `robot.hpp/cpp` | `Robot.java` | TimedRobot main class |
| `generated/TunerConstants.h/cpp` | `generated/TunerConstants.java` | Tuner X swerve constants |
| `test/cpp/inpututiltest.cpp` | `test/.../InputUtilTest.java` | Ported 5 test cases |
| `luabot/luabot2.hpp` | — | Dropped (Lua-specific) |
| `sol/*.hpp` | — | Dropped (sol2 Lua bindings) |
| `test/cpp/configtest.cpp` | — | Dropped (tested Lua config loading) |
| `test/cpp/vistionbasicstest.cpp` | — | Dropped (C++ vision basic tests) |

---

## Architecture Changes

### 1. Configuration system: Lua → static constants

The C++ project used an embedded Lua interpreter (`sol2`) to load `robot/config.lua`
at runtime. This allowed hot-reloading config without recompiling. The Java port
replaces this with `Config.java`, a final class with only `public static final`
fields.

**C++ usage:**
```cpp
auto intake_voltage = config::get("intake_voltage").get<double>();
```

**Java equivalent:**
```java
double intakeVoltage = Config.INTAKE_VOLTAGE;
```

Every key from `config.lua` has a corresponding constant in `Config.java`. The
`Config.display()` method mirrors the C++ `config::display()` function.

### 2. Container pattern: inheritance → single class

The C++ `Container` used an abstract base class with two subclasses
(`JoystickContainer`, `GamepadContainer`) selected by a factory method
`Container::create()`. The Java port uses a single `RobotContainer` class with
two private methods:

```java
if (Config.GAMEPAD) {
    configureGamepadBindings();
} else {
    configureJoystickBindings();
}
```

### 3. Memory management: unique_ptr → direct fields

C++ subsystems were held as `std::unique_ptr<T>` members with explicit
construction in the container. Java subsystems are direct fields initialized
at declaration:

```java
private final Intake intake = new Intake();
private final Climber climber = new Climber();
private final Turret turret = new Turret();
```

### 4. Command ownership: CommandPtr → Command

C++ uses `frc2::CommandPtr` (move-only RAII wrapper). Java uses `Command`
(reference type, garbage collected). The `optional<CommandPtr>` pattern in C++
Robot becomes a nullable `Command` field in Java.

### 5. Units: C++ units library → WPILib Units class

C++ code uses `units::meters_per_second_t`, `0.75_tps`, etc. Java uses the
WPILib Units static import pattern:

```java
import static edu.wpi.first.units.Units.*;
double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
double maxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
```

---

## Subsystem-by-Subsystem Notes

### CommandSwerveDrivetrain

- **Generics**: `SwerveDrivetrain<TalonFX, TalonFX, CANcoder>` — parameterized
  by device types, not configuration types.
- **Constructors**: Use `TalonFX::new, TalonFX::new, CANcoder::new` as
  `DeviceConstructor` functional interface arguments (method references).
- **SysId routines**: Three variants (translation, steer, rotation) using the
  same `SysIdRoutine` builder pattern as C++.
- **PathPlanner**: Configured via `configurePathPlanner()` method with
  `PIDConstants(10, 0, 0)` for translation and `PIDConstants(7.5, 0, 0)` for
  rotation.
- **Operator perspective**: Alliance-aware rotation seeding in `periodic()`.
- **Simulation**: Background `Notifier` thread at 4ms for high-frequency sim
  updates.

### Intake

- Two Kraken X44 motors (top ID=14, bottom ID=15, both on "rio" CAN bus).
- Top motor: `CounterClockwise_Positive`; bottom motor: `Clockwise_Positive`.
- Commands: `startCommand()`, `stopCommand()`, `intakeCommand()` (with
  `finallyDo` stop), `ejectCommand()`, `stutterCommand()`.
- Stutter command uses timer-based 0.25s on/off cycling for
  `Config.INTAKE_STUTTER_LENGTH` seconds.

### Climber

- Single Kraken X60 motor (ID=1, "rio" bus) with 180:1 gear ratio.
- Software limits: forward=0.0 rotations, reverse=−3.109043 rotations.
- Commands: `climbCommand()`, `lowerCommand()`, `disableSoftLimitsCommand()`,
  `enableSoftLimitsAndResetCommand()`.

### Turret

- Three motors on "rio" bus:
  - Rotation (ID=19): Position control, PID(24, 0, 0.2), soft-limited
  - Shooter (ID=16): Velocity control at 54 rps, PID(0.3, 0, 0, kV=0.12)
  - Uptake (ID=21): Velocity control at 100 rps, PID(0.1, 0, 0, kV=0.12)
- Position hold latch: When operator input drops below threshold, the turret
  locks to its current position using the position control request.
- Shoot command: If shooter is already at speed (warm), fires immediately;
  otherwise spins up, waits for ready, then feeds.

### Vision / VisionMulti

- `Vision.java` defines constants, the `VisionMeasurement` record, and the
  `VisionIO` interface.
- Four cameras: FL, FR, BL, BR with unique `Transform3d` robot-to-camera
  transforms.
- `VisionMulti` implements `VisionIO` with per-camera `PhotonPoseEstimator`.
- Gating: no-targets, latency > 0.5s, ambiguity > 0.3, out-of-field-bounds.
- Std devs scale linearly with distance: `xy = 0.01 + distance * 0.05`.
- `BOT_VISION = false` by default (same as C++ `#define BOT_VISION 0`).

### Telemetry

- Publishes swerve drive state (pose, speeds, module states) to NetworkTables.
- Renders a `Mechanism2d` widget with per-module ligaments showing speed and
  direction.
- Phoenix 6 `SignalLogger` integration for time-series logging.

---

## API Differences & Gotchas

These are the issues encountered during the port that required non-obvious
API translations. Each was discovered through compilation errors and resolved
by inspecting the actual 2026 jar files.

### 1. SwerveDrivetrain generics

**C++ (no generics):**
```cpp
class CommandSwerveDrivetrain : public swerve::SwerveDrivetrain { ... }
```

**Java (parameterized by device type, not config type):**
```java
public class CommandSwerveDrivetrain
    extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder>
    implements Subsystem { ... }
```

The generic parameters are the hardware device classes, not their configuration
classes.

### 2. SwerveDrivetrain constructors — DeviceConstructor

**C++ (implicit):**
```cpp
CommandSwerveDrivetrain(SwerveDrivetrainConstants const& constants, ...)
```

**Java (requires factory method references):**
```java
public CommandSwerveDrivetrain(
    SwerveDrivetrainConstants constants,
    SwerveModuleConstants<?, ?, ?>... modules
) {
    super(TalonFX::new, TalonFX::new, CANcoder::new, constants, modules);
}
```

The Java `SwerveDrivetrain` superclass requires `DeviceConstructor` functional
interface instances (method references to `TalonFX::new` and `CANcoder::new`)
that the C++ version handles implicitly.

### 3. TalonFX constructor — CANBus object vs String

**Deprecated:**
```java
new TalonFX(14, "rio")   // String-based constructor marked @Deprecated(forRemoval)
```

**Preferred:**
```java
new TalonFX(14, new CANBus("rio"))
```

All subsystem motor constructors in the Java port use the `CANBus` object form.

### 4. AprilTagFields enum casing

**Wrong (common assumption):**
```java
AprilTagFields.k2026RebuiltAndyMark   // capital 'M' — does not compile
```

**Correct:**
```java
AprilTagFields.k2026RebuiltAndymark   // lowercase 'm'
```

### 5. PhotonPipelineResult latency

**C++ (units-based):**
```cpp
result.GetLatency().value()  // returns seconds via units::second_t
```

**Java (no getLatency() method in 2026):**
```java
result.metadata.getLatencyMillis() / 1000.0   // milliseconds field
```

The `getLatency()` method does not exist on `PhotonPipelineResult` in
photonlib 2026.2.1.

### 6. PhotonPoseEstimator — deprecated constructor & update()

**Deprecated (forRemoval, since 2026):**
```java
new PhotonPoseEstimator(layout, PoseStrategy.LOWEST_AMBIGUITY, robotToCamera)
estimator.update(result)
```

**Current:**
```java
new PhotonPoseEstimator(layout, robotToCamera)
estimator.estimateLowestAmbiguityPose(result)
```

The strategy enum is removed from construction; you call the specific
estimation method directly. Available methods:
- `estimateLowestAmbiguityPose()`
- `estimateClosestToCameraHeightPose()`
- `estimateCoprocMultiTagPose()`
- `estimateAverageBestTargetsPose()`
- etc.

### 7. Vision std devs — Matrix<N3,N1> from double[]

**C++ (implicit conversion):**
```cpp
drivetrain.AddVisionMeasurement(pose, timestamp, {xy, xy, theta});
```

**Java (requires VecBuilder):**
```java
drivetrain.addVisionMeasurement(pose, timestamp, VecBuilder.fill(xy, xy, theta));
```

Java's `addVisionMeasurement` expects `Matrix<N3, N1>`, which cannot be
constructed from a raw `double[]`.

### 8. Command.schedule() → CommandScheduler

**Deprecated (forRemoval, since 2025):**
```java
autoCommand.schedule();
```

**Current:**
```java
CommandScheduler.getInstance().schedule(autoCommand);
```

---

## What Was Dropped

| C++ Feature | Reason |
|---|---|
| Lua scripting runtime (`sol2`, `scripting.hpp/cpp`) | Java has no equivalent embedded Lua; all config is now compile-time constants |
| `LuaBot.json` vendordep | Only needed for C++ Lua module loading |
| `luabot/luabot2.hpp` | Lua REPL / hot-reload glue |
| `sol/*.hpp` headers | sol2 C++ Lua binding library |
| `robot/config.lua` deploy artifact | Config is compiled into `Config.java`; no runtime file needed |
| `extractLuaModules` Gradle task | Downloaded Lua modules from Maven — not applicable |
| `BOT_DUMB_CAMERA` / `cameraThread()` | C++ background camera thread; not ported (vision handled differently) |
| `BOT_TRACE_SUBSYSTEMS` | C++ conditional debug tracing; can be re-added if needed |
| `configtest.cpp` | Tested Lua config loading — no longer applicable |
| `vistionbasicstest.cpp` | C++ single-camera vision tests; can be re-added against Java Vision classes |
| SSH `sshAntTask` deploy config | Java deploy uses standard FRCJavaArtifact |

### Possible future additions

- **Runtime config**: Replace `Config.java` with a JSON/YAML config file loaded
  from `src/main/deploy/` for runtime configurability without recompiling.
- **Vision tests**: Port `vistionbasicstest.cpp` to JUnit 5 tests against
  `Vision.java` and `VisionMulti.java`.
- **Subsystem tracing**: Add a `BOT_TRACE_SUBSYSTEMS`-equivalent flag with
  SmartDashboard telemetry in each subsystem's `periodic()`.

---

## Testing

### Ported tests

| C++ Test | Java Test | Status |
|---|---|---|
| `inpututiltest.cpp` | `InputUtilTest.java` | ✅ 5/5 passing |

**Test cases:**
1. `applyCurveReturnsZeroInsideDeadband` — values within deadband return 0
2. `applyCurveReturnsFullAtExtremes` — ±1.0 input returns ±1.0 output
3. `applyCurveLinearExponent` — exponent=1.0 produces linear scaling
4. `applyCurveWithZeroDeadbandLinear` — zero deadband with linear exponent
5. `applyCurvePreservesSign` — positive/negative symmetry

### Not ported

- `configtest.cpp` — tested Lua `sol::state` config loading (not applicable)
- `vistionbasicstest.cpp` — tested single-camera PhotonVision basics

---

## Build Verification

The final build produces **zero errors and zero warnings**:

```
> Task :compileJava
> Task :classes
> Task :jar
> Task :assemble
> Task :compileTestJava
> Task :test

InputUtilTest > applyCurveReturnsFullAtExtremes() PASSED
InputUtilTest > applyCurveLinearExponent() PASSED
InputUtilTest > applyCurveReturnsZeroInsideDeadband() PASSED
InputUtilTest > applyCurveWithZeroDeadbandLinear() PASSED
InputUtilTest > applyCurvePreservesSign() PASSED

> Task :check
> Task :build

BUILD SUCCESSFUL
```

### Compilation issues resolved during port

| # | Error | Root Cause | Fix |
|---|---|---|---|
| 1 | `SwerveDrivetrain` generic args wrong | Used config types instead of device types | `<TalonFX, TalonFX, CANcoder>` |
| 2 | Constructor missing `DeviceConstructor` args | Java requires factory method refs | Added `TalonFX::new, TalonFX::new, CANcoder::new` |
| 3 | `double[]` incompatible with `Matrix<N3,N1>` | Implicit conversion doesn't exist in Java | `VecBuilder.fill(x, y, z)` |
| 4 | `k2026RebuiltAndyMark` not found | Wrong casing | `k2026RebuiltAndymark` |
| 5 | `getLatency()` not found | API doesn't exist in photonlib 2026 | `metadata.getLatencyMillis() / 1000.0` |
| 6 | `TalonFX(int, String)` deprecated | String CAN bus constructor | `new TalonFX(id, new CANBus("bus"))` |
| 7 | `PhotonPoseEstimator` constructor deprecated | PoseStrategy removed from ctor | Two-arg constructor + `estimateLowestAmbiguityPose()` |
| 8 | `Command.schedule()` deprecated | Instance method removed | `CommandScheduler.getInstance().schedule(cmd)` |
