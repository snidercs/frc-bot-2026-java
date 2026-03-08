// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Config;
import frc.robot.Vision;

/**
 * Turret shooter subsystem with auto-aim capability.
 *
 * Controls a turret rotation motor for aiming and shooter flywheel motor(s).
 * Supports both auto-aim mode (using robot pose from vision) and manual control.
 *
 * Ported from turret.hpp/cpp in the C++ project.
 */
public class Turret extends SubsystemBase {

    // Rotation motor (positions turret)
    private final TalonFX rotationMotor =
        new TalonFX(Config.TURRET_ROTATION_DEVICE_ID, new CANBus(Config.TURRET_ROTATION_CAN_BUS));

    // Shooter flywheel motor(s)
    private final TalonFX shooterMotor =
        new TalonFX(Config.TURRET_SHOOTER_DEVICE_ID, new CANBus(Config.TURRET_SHOOTER_CAN_BUS));

    // Uptake motor (feeds game pieces into shooter)
    private final TalonFX uptakeMotor =
        new TalonFX(Config.TURRET_UPTAKE_DEVICE_ID, new CANBus(Config.TURRET_UPTAKE_CAN_BUS));

    // Control requests (reusable)
    private final PositionVoltage positionRequest = new PositionVoltage(0);
    private final VelocityVoltage rotationVelocityRequest = new VelocityVoltage(0);
    private final VelocityVoltage shooterVelocityRequest = new VelocityVoltage(0);
    private final VelocityVoltage uptakeVelocityRequest = new VelocityVoltage(0);
    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);

    // State
    private boolean autoAimEnabled = false;
    private double targetAngleDeg = 0;

    // Cached sensor values (updated in periodic to avoid redundant CAN reads)
    private double cachedAngleDeg = 0;
    private double cachedShooterVelocityRps = 0;
    private double cachedRotationVelocityRps = 0;
    private double cachedMotorVoltage = 0;
    private double cachedMotorCurrent = 0;

    // Position hold state (for locking when no operator input)
    private boolean isHoldingPosition = false;
    private double holdPositionRotations = 0;

    // Constants
    private static final double kShooterVelocityRps = 54;
    private static final double kShooterToleranceRps = 5;
    private static final double kUptakeVelocityRps = 100;
    private static final double kAngleToleranceDeg = 2;
    private static final double kManualRotationSpeedRps = 0.3;

    // Gear ratio from motor to turret (motor rotations per turret rotation)
    private static final double kRotationGearRatio = 100.0; // TODO: measure actual ratio

    public Turret() {
        setName("Turret");
        configureMotors();
    }

    private void configureMotors() {
        // Rotation motor configuration (position control for aiming)
        TalonFXConfiguration rotationConfig = new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimit(Amps.of(30))
                    .withSupplyCurrentLimitEnable(true)
                    .withStatorCurrentLimit(Amps.of(60))
                    .withStatorCurrentLimitEnable(true)
            )
            .withVoltage(
                new VoltageConfigs()
                    .withPeakForwardVoltage(Volts.of(12))
                    .withPeakReverseVoltage(Volts.of(-12))
            )
            .withSlot0(
                new Slot0Configs()
                    .withKP(24.0)
                    .withKI(0.0)
                    .withKD(0.2)
                    .withKV(0.12)
            )
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake)
            )
            .withFeedback(
                new FeedbackConfigs()
                    .withSensorToMechanismRatio(10.0)
            )
            .withSoftwareLimitSwitch(
                new SoftwareLimitSwitchConfigs()
                    .withForwardSoftLimitEnable(true)
                    .withForwardSoftLimitThreshold(Rotations.of(0.25))
                    .withReverseSoftLimitEnable(true)
                    .withReverseSoftLimitThreshold(Rotations.of(-0.05542))
            );

        // Shooter motor configuration (velocity control for flywheel)
        TalonFXConfiguration shooterConfig = new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimit(Amps.of(40))
                    .withSupplyCurrentLimitEnable(true)
                    .withStatorCurrentLimit(Amps.of(80))
                    .withStatorCurrentLimitEnable(true)
            )
            .withVoltage(
                new VoltageConfigs()
                    .withPeakForwardVoltage(Volts.of(12))
                    .withPeakReverseVoltage(Volts.of(-12))
            )
            .withSlot0(
                new Slot0Configs()
                    .withKP(0.2)
                    .withKI(0.0)
                    .withKD(0.0)
                    .withKV(0.12)
                    .withKS(0.25)
            )
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Coast)
            );

        // Uptake motor configuration (velocity control for feeding)
        TalonFXConfiguration uptakeConfig = new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimit(Amps.of(30))
                    .withSupplyCurrentLimitEnable(true)
                    .withStatorCurrentLimit(Amps.of(60))
                    .withStatorCurrentLimitEnable(true)
            )
            .withVoltage(
                new VoltageConfigs()
                    .withPeakForwardVoltage(Volts.of(12))
                    .withPeakReverseVoltage(Volts.of(-12))
            )
            .withSlot0(
                new Slot0Configs()
                    .withKP(0.2)
                    .withKI(0.0)
                    .withKD(0.0)
                    .withKV(0.12)
                    .withKS(0.2)
            )
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Coast)
            );

        // Apply configs
        rotationMotor.getConfigurator().apply(rotationConfig);
        rotationMotor.setNeutralMode(NeutralModeValue.Brake);
        rotationMotor.setPosition(Rotations.of(0.25));

        shooterMotor.getConfigurator().apply(shooterConfig);
        uptakeMotor.getConfigurator().apply(uptakeConfig);

        // Configure status signal update frequencies
        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            rotationMotor.getPosition(),
            rotationMotor.getVelocity(),
            rotationMotor.getSupplyCurrent(),
            shooterMotor.getVelocity(),
            shooterMotor.getSupplyCurrent(),
            uptakeMotor.getVelocity(),
            uptakeMotor.getSupplyCurrent()
        );

        // Optimize CAN bus utilization
        rotationMotor.optimizeBusUtilization();
        shooterMotor.optimizeBusUtilization();
        uptakeMotor.optimizeBusUtilization();
    }

    @Override
    public void periodic() {
        // Cache all sensor values once per cycle to minimize CAN bus reads
        cachedAngleDeg = rotationMotor.getPosition().getValue().in(Rotations) * 360.0;
        cachedShooterVelocityRps = shooterMotor.getVelocity().getValue().in(RotationsPerSecond);
        cachedRotationVelocityRps = rotationMotor.getVelocity().getValue().in(RotationsPerSecond);
        cachedMotorVoltage = rotationMotor.getMotorVoltage().getValue().in(Volts);
        cachedMotorCurrent = rotationMotor.getSupplyCurrent().getValue().in(Amps);

        // Telemetry (disabled by default)
        // SmartDashboard.putBoolean("Turret/Auto Aim Enabled", autoAimEnabled);
        // SmartDashboard.putNumber("Turret/Current Angle (deg)", cachedAngleDeg);
        // SmartDashboard.putNumber("Turret/Target Angle (deg)", targetAngleDeg);
    }

    // ── Manual control ───────────────────────────────────────────────────────

    public void setRotationVelocity(double rps) {
        rotationMotor.setControl(rotationVelocityRequest.withVelocity(RotationsPerSecond.of(rps)));
    }

    public void setRotationDutyCycle(double dutyCycle) {
        dutyCycle = Math.max(-0.1, Math.min(0.1, dutyCycle));
        SmartDashboard.putNumber("Turret/Commanded Duty Cycle", dutyCycle);
        rotationMotor.setControl(dutyCycleRequest.withOutput(dutyCycle));
    }

    /**
     * Handles hold/manual control. When there is active operator input,
     * uses percent output. When input is zero, latches and holds current position.
     */
    public void updateRotationControl(double operatorCommand) {
        final double kDeadband = 0.05;
        boolean hasInput = Math.abs(operatorCommand) > kDeadband;

        if (hasInput) {
            isHoldingPosition = false;
            double kMaxOutput = 0.3;
            double scaledOutput = Math.max(-1.0, Math.min(1.0, operatorCommand)) * kMaxOutput;
            rotationMotor.setControl(dutyCycleRequest.withOutput(scaledOutput));
        } else {
            if (!isHoldingPosition) {
                holdPositionRotations = rotationMotor.getPosition().getValue().in(Rotations);
                isHoldingPosition = true;
            }
            rotationMotor.setControl(positionRequest.withPosition(Rotations.of(holdPositionRotations)));
        }

        SmartDashboard.putBoolean("Turret/Holding Position", isHoldingPosition);
        SmartDashboard.putNumber("Turret/Hold Position (turns)", holdPositionRotations);
    }

    public void setShooterVelocity(double rps) {
        shooterMotor.setControl(shooterVelocityRequest.withVelocity(RotationsPerSecond.of(rps)));
    }

    public void stopRotation() {
        rotationMotor.setControl(voltageRequest.withOutput(Volts.of(0)));
    }

    public void stopShooter() {
        shooterMotor.setControl(voltageRequest.withOutput(Volts.of(0)));
    }

    public void stopUptake() {
        uptakeMotor.setControl(voltageRequest.withOutput(Volts.of(0)));
    }

    public void stop() {
        stopRotation();
        stopShooter();
        stopUptake();
    }

    // ── Auto-aim ─────────────────────────────────────────────────────────────

    public void enableAutoAim() { autoAimEnabled = true; }
    public void disableAutoAim() { autoAimEnabled = false; }
    public boolean isAutoAimEnabled() { return autoAimEnabled; }

    public void setTargetAngle(double angleDeg) {
        targetAngleDeg = angleDeg;
        double targetRotations = angleDeg / 360.0;
        rotationMotor.setControl(positionRequest.withPosition(Rotations.of(targetRotations)));
    }

    // ── Status ───────────────────────────────────────────────────────────────

    public double getCurrentAngleDeg() { return cachedAngleDeg; }
    public double getShooterVelocityRps() { return cachedShooterVelocityRps; }

    public boolean isAtTarget() {
        return Math.abs(getCurrentAngleDeg() - targetAngleDeg) < kAngleToleranceDeg;
    }

    public boolean isShooterReady() {
        return Math.abs(getShooterVelocityRps() - kShooterVelocityRps) < kShooterToleranceRps;
    }

    // ── Private helpers ──────────────────────────────────────────────────────

    private double computeAimAngle(Pose2d robotPose, Pose2d targetPose) {
        Translation2d robotToTarget = targetPose.getTranslation().minus(robotPose.getTranslation());
        Translation2d robotToTargetInRobotFrame = robotToTarget.rotateBy(robotPose.getRotation().unaryMinus());
        Translation2d turretPivot = Vision.TURRET_PIVOT_IN_ROBOT;
        Translation2d pivotToTarget = robotToTargetInRobotFrame.minus(turretPivot);
        return Math.toDegrees(Math.atan2(pivotToTarget.getY(), pivotToTarget.getX()));
    }

    // ── Command factories ────────────────────────────────────────────────────

    public Command aimAtTargetCommand(Supplier<Pose2d> robotPoseSupplier,
                                      Supplier<Pose2d> targetPoseSupplier) {
        return run(() -> {
            if (autoAimEnabled) {
                double aimAngle = computeAimAngle(robotPoseSupplier.get(), targetPoseSupplier.get());
                setTargetAngle(aimAngle);
            }
        }).withName("AimAtTarget");
    }

    public Command manualRotateCommand(DoubleSupplier speedSupplier) {
        return run(() -> {
            if (!autoAimEnabled) {
                updateRotationControl(speedSupplier.getAsDouble());
            }
        }).withName("ManualRotate");
    }

    public Command spinUpCommand() {
        return run(() -> setShooterVelocity(kShooterVelocityRps))
            .withName("SpinUp");
    }

    public Command stopCommand() {
        return runOnce(this::stop)
            .withName("StopTurret");
    }

    /**
     * Turns on the shooter and uptake. If already at speed, starts uptake immediately;
     * otherwise spins up first and waits until ready.
     */
    public Command shooterOnCommand() {
        Command warm = Commands.sequence(
            runOnce(() -> setShooterVelocity(kShooterVelocityRps)),
            runOnce(() -> uptakeMotor.setControl(
                uptakeVelocityRequest.withVelocity(RotationsPerSecond.of(kUptakeVelocityRps))))
        );

        Command cold = Commands.sequence(
            runOnce(() -> setShooterVelocity(kShooterVelocityRps)),
            Commands.waitUntil(this::isShooterReady),
            runOnce(() -> uptakeMotor.setControl(
                uptakeVelocityRequest.withVelocity(RotationsPerSecond.of(kUptakeVelocityRps))))
        );

        return Commands.either(warm, cold, this::isShooterReady)
            .withName("ShooterOn");
    }

    public Command shooterOffCommand() {
        return runOnce(() -> {
            stopUptake();
            stopShooter();
        }).withName("ShooterOff");
    }

    public Command shootCommand() {
        return Commands.sequence(
            shooterOnCommand(),
            Commands.idle()
        )
        .finallyDo(interrupted -> {
            stopUptake();
            stopShooter();
        })
        .withName("ManualShoot");
    }

    public Command calibrateRotationZero() {
        return runOnce(() -> {
            stopRotation();
            holdPositionRotations = 0;
            rotationMotor.setPosition(Rotations.of(0));
            isHoldingPosition = true;
        }).withName("ZeroRotation");
    }

    // ── Aim-at-Hub mode ──────────────────────────────────────────────────────

    /**
     * Continuously aims the turret at the Hub for the current alliance.
     * The turret tracks the Hub every cycle using the live robot pose.
     * Also spins up the shooter so the robot is ready to fire immediately.
     *
     * @param robotPoseSupplier supplies the current field-relative robot pose
     * @return a command that runs until cancelled
     */
    public Command aimAtHubCommand(Supplier<Pose2d> robotPoseSupplier) {
        return run(() -> {
            Pose2d targetPose = new Pose2d(Vision.hubPosition(), new Rotation2d());
            double aimAngle = computeAimAngle(robotPoseSupplier.get(), targetPose);
            setTargetAngle(aimAngle);
            setShooterVelocity(kShooterVelocityRps);
        })
        .finallyDo(interrupted -> {
            stopShooter();
        })
        .withName("AimAtHub");
    }

    /**
     * Returns true when the turret is aimed at the Hub and the shooter is at speed.
     */
    public boolean isReadyToShootHub() {
        return isAtTarget() && isShooterReady();
    }

    /**
     * Aims at the Hub and fires once ready (uptake feeds the game piece).
     * Stops shooter and uptake when cancelled.
     *
     * @param robotPoseSupplier supplies the current field-relative robot pose
     * @return a command that runs until cancelled
     */
    public Command aimAndShootHubCommand(Supplier<Pose2d> robotPoseSupplier) {
        return Commands.sequence(
            // Phase 1: aim and spin up until on-target and at speed
            run(() -> {
                Pose2d targetPose = new Pose2d(Vision.hubPosition(), new Rotation2d());
                double aimAngle = computeAimAngle(robotPoseSupplier.get(), targetPose);
                setTargetAngle(aimAngle);
                setShooterVelocity(kShooterVelocityRps);
            }).until(this::isReadyToShootHub),
            // Phase 2: feed game piece through uptake while continuing to aim
            run(() -> {
                Pose2d targetPose = new Pose2d(Vision.hubPosition(), new Rotation2d());
                double aimAngle = computeAimAngle(robotPoseSupplier.get(), targetPose);
                setTargetAngle(aimAngle);
                setShooterVelocity(kShooterVelocityRps);
                uptakeMotor.setControl(
                    uptakeVelocityRequest.withVelocity(RotationsPerSecond.of(kUptakeVelocityRps)));
            })
        )
        .finallyDo(interrupted -> {
            stopUptake();
            stopShooter();
        })
        .withName("AimAndShootHub");
    }

    // ── Pass-to-Tower mode ───────────────────────────────────────────────────

    /** Shooter velocity for passing fuel (lower than a full Hub shot). */
    private static final double kPassVelocityRps = 35;

    /**
     * Continuously aims the turret at the tower (pass target) for the current
     * alliance and spins the shooter at a reduced pass speed.
     * Use this when you want to lob fuel to a teammate or staging area
     * for later Hub shots.
     *
     * @param robotPoseSupplier supplies the current field-relative robot pose
     * @return a command that runs until cancelled
     */
    public Command aimPassCommand(Supplier<Pose2d> robotPoseSupplier) {
        return run(() -> {
            Pose2d robotPose = robotPoseSupplier.get();
            Translation2d tower = Vision.towerPosition(robotPose.getY());
            Pose2d targetPose = new Pose2d(tower, new Rotation2d());
            double aimAngle = computeAimAngle(robotPose, targetPose);
            setTargetAngle(aimAngle);
            setShooterVelocity(kPassVelocityRps);
        })
        .finallyDo(interrupted -> {
            stopShooter();
        })
        .withName("AimPass");
    }

    /**
     * Returns true when the turret is aimed at the pass target and the shooter
     * is at the pass speed.
     */
    public boolean isReadyToPass() {
        return isAtTarget()
            && Math.abs(getShooterVelocityRps() - kPassVelocityRps) < kShooterToleranceRps;
    }

    /**
     * Aims at the tower and passes fuel once ready.
     * Stops shooter and uptake when cancelled.
     *
     * @param robotPoseSupplier supplies the current field-relative robot pose
     * @return a command that runs until cancelled
     */
    public Command aimAndPassCommand(Supplier<Pose2d> robotPoseSupplier) {
        return Commands.sequence(
            // Phase 1: aim and spin up at pass speed
            run(() -> {
                Pose2d robotPose = robotPoseSupplier.get();
                Translation2d tower = Vision.towerPosition(robotPose.getY());
                Pose2d targetPose = new Pose2d(tower, new Rotation2d());
                double aimAngle = computeAimAngle(robotPose, targetPose);
                setTargetAngle(aimAngle);
                setShooterVelocity(kPassVelocityRps);
            }).until(this::isReadyToPass),
            // Phase 2: feed game piece
            run(() -> {
                Pose2d robotPose = robotPoseSupplier.get();
                Translation2d tower = Vision.towerPosition(robotPose.getY());
                Pose2d targetPose = new Pose2d(tower, new Rotation2d());
                double aimAngle = computeAimAngle(robotPose, targetPose);
                setTargetAngle(aimAngle);
                setShooterVelocity(kPassVelocityRps);
                uptakeMotor.setControl(
                    uptakeVelocityRequest.withVelocity(RotationsPerSecond.of(kUptakeVelocityRps)));
            })
        )
        .finallyDo(interrupted -> {
            stopUptake();
            stopShooter();
        })
        .withName("AimAndPass");
    }
}
