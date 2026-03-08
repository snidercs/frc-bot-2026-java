// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Config;

/**
 * Real TalonFX implementation of {@link IntakeIO}.
 *
 * Two Kraken X44 motors (top and bottom) running voltage control.
 */
public class IntakeIOTalonFX implements IntakeIO {

    private final TalonFX topMotor =
        new TalonFX(Config.INTAKE_TOP_DEVICE_ID, new CANBus(Config.INTAKE_TOP_CAN_BUS));
    private final TalonFX bottomMotor =
        new TalonFX(Config.INTAKE_BOTTOM_DEVICE_ID, new CANBus(Config.INTAKE_BOTTOM_CAN_BUS));

    private final VoltageOut voltageRequest = new VoltageOut(0);

    // NOTE: Changes to motor config should be reflected in
    //       README.md → "Power-Up Initialization".
    public IntakeIOTalonFX() {
        // Top motor configuration
        TalonFXConfiguration topConfig = new TalonFXConfiguration()
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
                    .withKP(0.1)
                    .withKI(0.0)
                    .withKD(0.0)
                    .withKV(0.12)
            )
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Coast)
            );

        // Bottom motor configuration (inverted)
        TalonFXConfiguration bottomConfig = new TalonFXConfiguration()
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
                    .withKP(0.1)
                    .withKI(0.0)
                    .withKD(0.0)
                    .withKV(0.12)
            )
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Coast)
            );

        // Apply configs
        topMotor.getConfigurator().apply(topConfig);
        bottomMotor.getConfigurator().apply(bottomConfig);

        // Configure status signal update frequencies to prevent CAN stale errors
        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            topMotor.getVelocity(),
            topMotor.getSupplyCurrent(),
            bottomMotor.getVelocity(),
            bottomMotor.getSupplyCurrent()
        );

        // Optimize CAN bus utilization
        topMotor.optimizeBusUtilization();
        bottomMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.topVelocityRps = topMotor.getVelocity().getValue().in(RotationsPerSecond);
        inputs.bottomVelocityRps = bottomMotor.getVelocity().getValue().in(RotationsPerSecond);
        inputs.topCurrentAmps = topMotor.getSupplyCurrent().getValue().in(Amps);
        inputs.bottomCurrentAmps = bottomMotor.getSupplyCurrent().getValue().in(Amps);
    }

    @Override
    public void setVoltage(double voltage) {
        topMotor.setControl(voltageRequest.withOutput(Volts.of(voltage)));
        bottomMotor.setControl(voltageRequest.withOutput(Volts.of(voltage)));
    }

    @Override
    public void stop() {
        topMotor.setControl(voltageRequest.withOutput(Volts.of(0)));
        bottomMotor.setControl(voltageRequest.withOutput(Volts.of(0)));
    }
}
