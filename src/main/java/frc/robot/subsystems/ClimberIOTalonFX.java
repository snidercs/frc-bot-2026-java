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

import frc.robot.Config;

/**
 * Real TalonFX implementation of {@link ClimberIO}.
 *
 * Single Kraken X60 motor with 180:1 gear ratio and software-limited travel.
 */
public class ClimberIOTalonFX implements ClimberIO {

    private final TalonFX motor =
        new TalonFX(Config.CLIMBER_DEVICE_ID, new CANBus(Config.CLIMBER_CAN_BUS));

    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0.0);

    // NOTE: Changes to motor config or zeroing should be reflected in
    //       README.md → "Power-Up Initialization".
    public ClimberIOTalonFX() {
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
                    .withForwardSoftLimitThreshold(Rotations.of(Climber.kForwardSoftLimit))
                    .withReverseSoftLimitEnable(true)
                    .withReverseSoftLimitThreshold(Rotations.of(Climber.kReverseSoftLimit))
            );

        motor.getConfigurator().apply(config);
        motor.setPosition(Rotations.of(0.0));

        // Configure status signal update frequencies
        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            motor.getPosition(),
            motor.getVelocity(),
            motor.getSupplyCurrent(),
            motor.getMotorVoltage()
        );
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.velocityRps = motor.getVelocity().getValue().in(RotationsPerSecond);
        inputs.positionRotations = motor.getPosition().getValue().in(Rotations);
        inputs.currentAmps = motor.getSupplyCurrent().getValue().in(Amps);
        inputs.voltage = motor.getMotorVoltage().getValue().in(Volts);
    }

    @Override
    public void setDutyCycle(double dutyCycle) {
        motor.setControl(dutyCycleRequest.withOutput(dutyCycle));
    }

    @Override
    public void stop() {
        motor.setControl(dutyCycleRequest.withOutput(0.0));
    }

    @Override
    public void disableSoftLimits() {
        motor.getConfigurator().apply(
            new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitEnable(false)
                .withReverseSoftLimitEnable(false)
        );
    }

    @Override
    public void enableSoftLimitsAndResetPosition(double forwardLimit, double reverseLimit) {
        motor.setPosition(Rotations.of(0.0));
        motor.getConfigurator().apply(
            new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitEnable(true)
                .withForwardSoftLimitThreshold(Rotations.of(forwardLimit))
                .withReverseSoftLimitEnable(true)
                .withReverseSoftLimitThreshold(Rotations.of(reverseLimit))
        );
    }
}
