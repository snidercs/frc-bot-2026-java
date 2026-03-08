// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Config;

/**
 * Climber subsystem with a single Kraken X60 motor.
 *
 * Ported from climber.hpp/cpp in the C++ project.
 */
public class Climber extends SubsystemBase {

    // Single Kraken x60 motor for climber
    private final TalonFX m_motor = new TalonFX(Config.CLIMBER_DEVICE_ID, new CANBus(Config.CLIMBER_CAN_BUS));

    // Control request (reusable)
    private final DutyCycleOut m_dutyCycleRequest = new DutyCycleOut(0.0);

    // Constants
    private static final double kClimbDutyCycle = 0.99;    // Positive = down/climbing
    private static final double kLowerDutyCycle = -0.99;   // Negative = up/lowering
    private static final double kForwardSoftLimit = 0.0;   // rotations
    private static final double kReverseSoftLimit = -3.109043; // rotations

    public Climber() {
        setName("Climber");
        configureMotor();
    }

    // NOTE: Changes to motor config or zeroing should be reflected in
    //       README.md → "Power-Up Initialization".
    private void configureMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration()
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
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake)
            )
            .withFeedback(
                new FeedbackConfigs()
                    .withSensorToMechanismRatio(180.0)
            )
            .withSoftwareLimitSwitch(
                new SoftwareLimitSwitchConfigs()
                    .withForwardSoftLimitEnable(true)
                    .withForwardSoftLimitThreshold(Rotations.of(kForwardSoftLimit))
                    .withReverseSoftLimitEnable(true)
                    .withReverseSoftLimitThreshold(Rotations.of(kReverseSoftLimit))
            );

        m_motor.getConfigurator().apply(config);
        m_motor.setPosition(Rotations.of(0.0));

        // Configure status signal update frequencies
        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            m_motor.getVelocity(),
            m_motor.getSupplyCurrent(),
            m_motor.getMotorVoltage()
        );
    }

    @Override
    public void periodic() {
        // Telemetry (disabled by default)
        // SmartDashboard.putNumber("Climber/Velocity (rps)", m_motor.getVelocity().getValue().in(RotationsPerSecond));
    }

    // Manual control
    public void setDutyCycle(double dutyCycle) {
        m_motor.setControl(m_dutyCycleRequest.withOutput(dutyCycle));
    }

    public void stop() {
        m_motor.setControl(m_dutyCycleRequest.withOutput(0.0));
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
        return runOnce(() -> {
            m_motor.getConfigurator().apply(
                new SoftwareLimitSwitchConfigs()
                    .withForwardSoftLimitEnable(false)
                    .withReverseSoftLimitEnable(false)
            );
        }).withName("DisableSoftLimits");
    }

    public Command enableSoftLimitsAndResetCommand() {
        return runOnce(() -> {
            m_motor.setPosition(Rotations.of(0.0));
            m_motor.getConfigurator().apply(
                new SoftwareLimitSwitchConfigs()
                    .withForwardSoftLimitEnable(true)
                    .withForwardSoftLimitThreshold(Rotations.of(kForwardSoftLimit))
                    .withReverseSoftLimitEnable(true)
                    .withReverseSoftLimitThreshold(Rotations.of(kReverseSoftLimit))
            );
        }).withName("EnableSoftLimitsAndReset");
    }
}
