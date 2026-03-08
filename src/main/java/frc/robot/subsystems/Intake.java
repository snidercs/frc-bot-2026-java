// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

import frc.robot.Config;

/**
 * Intake subsystem with two Kraken X44 motors (top and bottom).
 *
 * Hardware interaction is delegated to an {@link IntakeIO} implementation
 * so that sensor inputs are deterministically replayable via AdvantageKit.
 *
 * Ported from intake.hpp/cpp in the C++ project.
 */
public class Intake extends SubsystemBase {

    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    // Constants
    private final double kIntakeVoltage = Config.INTAKE_VOLTAGE;
    private static final double kEjectVoltage = 2.0;

    public Intake(IntakeIO io) {
        setName("Intake");
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
    }

    // Manual control
    public void setVoltage(double voltage) {
        io.setVoltage(voltage);
    }

    public void stop() {
        io.stop();
    }

    // Command factories
    public Command startCommand() {
        return runOnce(() -> setVoltage(kIntakeVoltage))
            .withName("IntakeStart");
    }

    public Command stopCommand() {
        return runOnce(this::stop)
            .withName("IntakeStop");
    }

    public Command intakeCommand() {
        return run(() -> setVoltage(kIntakeVoltage))
            .withName("Intake")
            .finallyDo(interrupted -> stop());
    }

    public Command ejectCommand() {
        return run(() -> setVoltage(kEjectVoltage))
            .withName("Eject")
            .finallyDo(interrupted -> stop());
    }

    /**
     * Runs the intake in 0.25s on / 0.25s off cycles, then stops.
     *
     * @param durationSeconds How long to run the stutter cycle before stopping.
     *                        If &lt;= 0, the duration is read from Config.INTAKE_STUTTER_LENGTH.
     */
    public Command stutterCommand(double durationSeconds) {
        if (durationSeconds <= 0) {
            durationSeconds = Config.INTAKE_STUTTER_LENGTH;
        }
        final double duration = durationSeconds;
        final double[] startTime = {0};

        return runOnce(() -> startTime[0] = Timer.getFPGATimestamp())
            .andThen(run(() -> {
                double t = Timer.getFPGATimestamp();
                if ((t % 0.5) < 0.25) {
                    setVoltage(kIntakeVoltage);
                } else {
                    stop();
                }
            })
            .until(() -> (Timer.getFPGATimestamp() - startTime[0]) >= duration))
            .finallyDo(interrupted -> stop())
            .withName("IntakeStutter");
    }

    /** Overload with default duration from config. */
    public Command stutterCommand() {
        return stutterCommand(0);
    }
}
