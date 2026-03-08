# Controls Reference — Team 9431 "Indy"

> **Robot:** Indy · **Season:** 2026 REBUILT · **Config mode:** `GAMEPAD = false` (dual flight sticks)
>
> Control mode is set in `Config.java`. When `GAMEPAD = true`, an Xbox controller
> layout is used instead (see [Gamepad Layout](#gamepad-layout-xbox-controller) below).

---

## Dual Flight Stick Layout (Default)

Two flight sticks are used: **Stick 0** (left hand, USB port 0) and **Stick 1** (right hand, USB port 1).

### Axes

| Stick | Axis | Function | Details |
|-------|------|----------|---------|
| Stick 0 | Axis 1 (Y) | Drive forward / backward | Field-centric, deadband 0.05, exponent 2.5 |
| Stick 0 | Axis 0 (X) | Drive strafe left / right | Field-centric, deadband 0.05, exponent 2.5 |
| Stick 1 | Axis 0 (X) | Rotate (yaw) | Field-centric, deadband 0.05, exponent 1.25 |
| Stick 0 | Axis 3 | Manual turret rotation | Gain 0.1, hold-position when released |

### Buttons — Stick 0 (Left Hand)

| Button | Trigger | Action | Subsystem |
|--------|---------|--------|-----------|
| 1 | While held | Lower climber | Climber |
| 5 | On press | Jitter front/back | Drivetrain |
| 6 | On press | Jitter left/right | Drivetrain |
| 8 | On press | Reset field-centric heading | Drivetrain |
| 16 | While held | Raise climber | Climber |
| 18 | While held | Run intake | Intake |

### Buttons — Stick 1 (Right Hand)

| Button | Trigger | Action | Subsystem |
|--------|---------|--------|-----------|
| 3 | On press | Zero turret rotation (calibrate) | Turret |
| 4 | On press / release | Disable climber soft limits (re-enables on release) | Climber |
| 9 | While held | **Aim at Hub** — turret tracks Hub, spins up shooter | Turret |
| 10 | While held | **Pass mode** — turret aims at nearest Tower, auto-fires at pass speed | Turret |
| 11 | While held | **Aim & shoot Hub** — turret tracks Hub, auto-fires when ready | Turret |
| 18 | While held | Manual shoot (spin up + feed) | Turret |
| 19 | While held | Eject intake (reverse) | Intake |

### Default Commands

| Subsystem | Default Behavior |
|-----------|-----------------|
| Drivetrain | Field-centric swerve drive from Stick 0 (translate) + Stick 1 (rotate) |
| Turret | Manual rotation from Stick 0 Axis 3 with position hold |
| Drivetrain (disabled) | Idle (modules coast) |

---

## Gamepad Layout (Xbox Controller)

Active when `Config.GAMEPAD = true`. Single Xbox controller on USB port 0.

### Axes

| Input | Function | Details |
|-------|----------|---------|
| Left Stick Y | Drive forward / backward | Deadband 0.05, exponent 2.5 |
| Left Stick X | Drive strafe left / right | Deadband 0.05, exponent 2.5 |
| Right Stick X | Rotate (yaw) | Deadband 0.05, exponent 2.5 |

### Buttons

| Button | Trigger | Action | Subsystem |
|--------|---------|--------|-----------|
| A | While held | Swerve brake (X-pattern) | Drivetrain |
| B | While held | Point wheels toward left stick direction | Drivetrain |
| Left Bumper | On press | Reset field-centric heading | Drivetrain |
| Right Bumper | While held | Run intake | Intake |
| Right Trigger | While held | Eject intake (reverse) | Intake |
| 1 | While held | Lower climber | Climber |
| 16 | While held | Raise climber | Climber |
| Back + Y | While held | SysId dynamic translation (forward) | Drivetrain |
| Back + X | While held | SysId dynamic translation (reverse) | Drivetrain |
| Start + Y | While held | SysId quasistatic translation (forward) | Drivetrain |
| Start + X | While held | SysId quasistatic translation (reverse) | Drivetrain |

> **Note:** The gamepad layout does not currently bind turret aim, shoot, or pass
> commands. Those are only available in dual-stick mode.

---

## Turret Aiming Modes

### Aim at Hub (Button 9)

Hold to continuously track the alliance Hub. The turret automatically computes
the angle from the robot's current field pose to the Hub center using the
onboard pose estimator. The shooter flywheel spins up so the robot is ready
to fire immediately. Release to stop the shooter.

### Aim & Shoot Hub (Button 11)

Hold to aim at the Hub and fire automatically. The command has two phases:

1. **Aim & spin-up** — turret tracks the Hub and the shooter spools to full speed (54 RPS).
2. **Fire** — once on-target and at speed, the uptake motor feeds fuel into the shooter.

Release at any time to stop the shooter and uptake.

### Pass to Tower (Button 10)

Hold to lob fuel to the nearest Tower for later Hub shots. The turret tracks
the closest Tower structure (upper or lower field edge, based on robot Y position)
and fires at a reduced pass speed (35 RPS). Like Aim & Shoot, the command
auto-feeds once aimed and at speed.

---

## Autonomous Named Commands

These commands are registered with PathPlanner and can be used in `.auto` routines:

| Name | Description |
|------|-------------|
| `shooterOn` | Spin up shooter, feed uptake when ready |
| `shooterOff` | Stop shooter and uptake |
| `turretStop` | Stop all turret motors |
| `intakeStart` | Run intake motors |
| `intakeStutter` | Run intake in stutter pattern |
| `intakeStop` | Stop intake motors |
| `driveJitter` | Wiggle robot front/back to seat game pieces |
| `aimAtHub` | Continuously aim turret at Hub + spin up |
| `aimAndShootHub` | Aim at Hub, auto-fire when ready |
| `aimPass` | Aim at nearest Tower + spin up at pass speed |
| `aimAndPass` | Aim at nearest Tower, auto-fire at pass speed |
| `climbAuto` | Run climber at full speed for up to 4 s (auto use) |
| `aimAndShootAuto` | Aim at Hub + auto-fire, 5 s timeout (auto use) |
| `intakeStutterShort` | Intake stutter pattern, 3 s timeout (auto use) |

### Autonomous Routines

Select via the **AutoChooser** widget on SmartDashboard. Default: `Backup-Shoot-Left`.

| Auto | Description |
|------|-------------|
| `Backup-Shoot-Left` | Back up left, shoot (no aim) |
| `Backup-Shoot-Mid-00` | Back up mid-low, shoot (no aim) |
| `Backup-Shoot-Mid-01` | Back up mid-high, shoot (no aim) |
| `Backup-Shoot-Right` | Back up right, shoot (no aim) |
| `Shoot-Climb-Left-Aimed` | Back up left → vision-aimed shoot → drive to upper cage → climb |
| `Shoot-Climb-Left-Simple` | Back up left → shoot (no aim) → drive to upper cage → climb |
| `Shoot-Climb-Mid-00-Aimed` | Back up mid-low → vision-aimed shoot → drive to mid cage → climb |
| `Shoot-Climb-Mid-00-Simple` | Back up mid-low → shoot (no aim) → drive to mid cage → climb |
| `Shoot-Climb-Mid-01-Aimed` | Back up mid-high → vision-aimed shoot → drive to mid cage → climb |
| `Shoot-Climb-Mid-01-Simple` | Back up mid-high → shoot (no aim) → drive to mid cage → climb |
| `Shoot-Climb-Right-Aimed` | Back up right → vision-aimed shoot → drive to lower cage → climb |
| `Shoot-Climb-Right-Simple` | Back up right → shoot (no aim) → drive to lower cage → climb |

> **Alliance flipping:** All paths are authored for Blue alliance. PathPlanner
> automatically mirrors them for Red alliance, so the robot drives to the correct
> alliance wall cage regardless of alliance color.

---

## Input Shaping

All axis inputs pass through `InputUtil.applyCurve(value, deadband, exponent)`:

| Parameter | Value | Notes |
|-----------|-------|-------|
| Drive deadband | 0.05 | Inputs below 5% are zeroed |
| Drive exponent | 2.5 | Aggressive curve for fine control at low speed |
| Rotate deadband | 0.05 | |
| Rotate exponent | 1.25 | Near-linear for predictable rotation |

---

## Configuration Reference

All button and axis indices are defined in `Config.java` and can be changed
without modifying `RobotContainer.java`.

| Constant | Value | Used By |
|----------|-------|---------|
| `HEADING_BUTTON_INDEX` | 8 | Stick 0 — reset heading |
| `INTAKE_TRIGGER_INDEX` | 18 | Stick 0 — intake |
| `INTAKE_EJECT_INDEX` | 19 | Stick 1 — eject |
| `CLIMBER_CLIMB_BUTTON_INDEX` | 16 | Stick 0 — climb up |
| `CLIMBER_LOWER_BUTTON_INDEX` | 1 | Stick 0 — climb down |
| `TURRET_ROTATION_AXIS_STICK` | 0 | Which stick for turret rotation |
| `TURRET_ROTATION_AXIS_INDEX` | 3 | Which axis on that stick |
| `TURRET_ROTATION_GAIN` | 0.1 | Sensitivity multiplier |
| `TURRET_AIM_BUTTON_INDEX` | 7 | (Reserved) |
| `TURRET_SHOOT_BUTTON_INDEX` | 18 | Stick 1 — manual shoot |
| `TURRET_AIM_HUB_BUTTON_INDEX` | 9 | Stick 1 — aim at Hub |
| `TURRET_AIM_PASS_BUTTON_INDEX` | 10 | Stick 1 — pass to Tower |
| `TURRET_FIRE_BUTTON_INDEX` | 11 | Stick 1 — aim & shoot Hub |
