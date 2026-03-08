// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

/**
 * Climber subsystem with a single Kraken X60 motor.
 *
 * Hardware interaction is delegated to a {@link ClimberIO} implementation
 * so that sensor inputs are deterministically replayable via AdvantageKit.
 *
 * Ported from climber.hpp/cpp in the C++ project.
 */
public class Climber extends SubsystemBase {

    private final ClimberIO io;
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

    // Constants
    private static final double kClimbDutyCycle = 0.99;    // Positive = down/climbing
    private static final double kLowerDutyCycle = -0.99;   // Negative = up/lowering
    static final double kForwardSoftLimit = 0.0;           // rotations
    static final double kReverseSoftLimit = -3.109043;     // rotations

    public Climber(ClimberIO io) {
        setName("Climber");
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Climber", inputs);
    }

    // Manual control
    public void setDutyCycle(double dutyCycle) {
        io.setDutyCycle(dutyCycle);
    }

    public void stop() {
        io.stop();
    }

    // Command factories
    public Command climbCommand() {
        return run(() -> setDutyCycle(kClimbDutyCycle))
            .withName("Climb")
            .finallyDo(interrupted -> stop());
    }

    public Command lowerCommand() {
        return run(() -> setDutyCycle(kLowerDutyCycle))
            .withName("Lower")
            .finallyDo(interrupted -> stop());
    }

    public Command stopCommand() {
        return runOnce(this::stop)
            .withName("StopClimber");
    }

    public Command disableSoftLimitsCommand() {
        return runOnce(() -> io.disableSoftLimits())
            .withName("DisableSoftLimits");
    }

    public Command enableSoftLimitsAndResetCommand() {
        return runOnce(() -> io.enableSoftLimitsAndResetPosition(kForwardSoftLimit, kReverseSoftLimit))
            .withName("EnableSoftLimitsAndReset");
    }
}
