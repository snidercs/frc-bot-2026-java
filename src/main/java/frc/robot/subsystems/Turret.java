// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import frc.robot.Vision;

/**
 * Turret shooter subsystem with auto-aim capability.
 *
 * Controls a turret rotation motor for aiming and shooter flywheel motor(s).
 * Supports both auto-aim mode (using robot pose from vision) and manual control.
 *
 * Hardware interaction is delegated to a {@link TurretIO} implementation
 * so that sensor inputs are deterministically replayable via AdvantageKit.
 *
 * Ported from turret.hpp/cpp in the C++ project.
 */
public class Turret extends SubsystemBase {

    private final TurretIO io;
    private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

    // Subsystem state (logged via @AutoLogOutput)
    @AutoLogOutput(key = "Turret/AutoAimEnabled")
    private boolean autoAimEnabled = false;
    @AutoLogOutput(key = "Turret/TargetAngleDeg")
    private double targetAngleDeg = 0;
    @AutoLogOutput(key = "Turret/CommandedDutyCycle")
    private double cachedDutyCycle = 0;

    // Position hold state (for locking when no operator input)
    @AutoLogOutput(key = "Turret/HoldingPosition")
    private boolean isHoldingPosition = false;
    @AutoLogOutput(key = "Turret/HoldPositionRotations")
    private double holdPositionRotations = 0;

    // Constants
    private static final double kShooterVelocityRps = 54;
    private static final double kShooterToleranceRps = 5;
    private static final double kUptakeVelocityRps = 100;
    private static final double kAngleToleranceDeg = 2;
    private static final double kManualRotationSpeedRps = 0.3;

    // Gear ratio from motor to turret (motor rotations per turret rotation)
    private static final double kRotationGearRatio = 100.0; // TODO: measure actual ratio

    public Turret(TurretIO io) {
        setName("Turret");
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Turret", inputs);
    }

    // ── Manual control ───────────────────────────────────────────────────────

    public void setRotationVelocity(double rps) {
        io.setRotationVelocity(rps);
    }

    public void setRotationDutyCycle(double dutyCycle) {
        dutyCycle = Math.max(-0.1, Math.min(0.1, dutyCycle));
        cachedDutyCycle = dutyCycle;
        io.setRotationDutyCycle(dutyCycle);
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
            io.setRotationDutyCycle(scaledOutput);
        } else {
            if (!isHoldingPosition) {
                holdPositionRotations = inputs.rotationPositionRotations;
                isHoldingPosition = true;
            }
            io.setRotationPosition(holdPositionRotations);
        }
    }

    public void setShooterVelocity(double rps) {
        io.setShooterVelocity(rps);
    }

    public void stopRotation() {
        io.stopRotation();
    }

    public void stopShooter() {
        io.stopShooter();
    }

    public void stopUptake() {
        io.stopUptake();
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
        io.setRotationPosition(targetRotations);
    }

    // ── Status ───────────────────────────────────────────────────────────────

    public double getCurrentAngleDeg() {
        return inputs.rotationPositionRotations * 360.0;
    }

    public double getShooterVelocityRps() {
        return inputs.shooterVelocityRps;
    }

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
            runOnce(() -> io.setUptakeVelocity(kUptakeVelocityRps))
        );

        Command cold = Commands.sequence(
            runOnce(() -> setShooterVelocity(kShooterVelocityRps)),
            Commands.waitUntil(this::isShooterReady),
            runOnce(() -> io.setUptakeVelocity(kUptakeVelocityRps))
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
            io.zeroRotationPosition(0);
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
                io.setUptakeVelocity(kUptakeVelocityRps);
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
                io.setUptakeVelocity(kUptakeVelocityRps);
            })
        )
        .finallyDo(interrupted -> {
            stopUptake();
            stopShooter();
        })
        .withName("AimAndPass");
    }
}
