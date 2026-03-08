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

import frc.robot.Config;

/**
 * Real TalonFX implementation of {@link TurretIO}.
 *
 * Three motors: rotation (position PID), shooter flywheel (velocity),
 * and uptake feeder (velocity). All on the {@code rio} CAN bus.
 */
public class TurretIOTalonFX implements TurretIO {

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

    // NOTE: Changes to motor config or zeroing should be reflected in
    //       README.md → "Power-Up Initialization".
    public TurretIOTalonFX() {
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
            rotationMotor.getMotorVoltage(),
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
    public void updateInputs(TurretIOInputs inputs) {
        inputs.rotationPositionRotations = rotationMotor.getPosition().getValue().in(Rotations);
        inputs.rotationVelocityRps = rotationMotor.getVelocity().getValue().in(RotationsPerSecond);
        inputs.rotationVoltage = rotationMotor.getMotorVoltage().getValue().in(Volts);
        inputs.rotationCurrentAmps = rotationMotor.getSupplyCurrent().getValue().in(Amps);
        inputs.shooterVelocityRps = shooterMotor.getVelocity().getValue().in(RotationsPerSecond);
    }

    @Override
    public void setRotationPosition(double rotations) {
        rotationMotor.setControl(positionRequest.withPosition(Rotations.of(rotations)));
    }

    @Override
    public void setRotationVelocity(double rps) {
        rotationMotor.setControl(rotationVelocityRequest.withVelocity(RotationsPerSecond.of(rps)));
    }

    @Override
    public void setRotationDutyCycle(double dutyCycle) {
        rotationMotor.setControl(dutyCycleRequest.withOutput(dutyCycle));
    }

    @Override
    public void setShooterVelocity(double rps) {
        shooterMotor.setControl(shooterVelocityRequest.withVelocity(RotationsPerSecond.of(rps)));
    }

    @Override
    public void setUptakeVelocity(double rps) {
        uptakeMotor.setControl(uptakeVelocityRequest.withVelocity(RotationsPerSecond.of(rps)));
    }

    @Override
    public void stopRotation() {
        rotationMotor.setControl(voltageRequest.withOutput(Volts.of(0)));
    }

    @Override
    public void stopShooter() {
        shooterMotor.setControl(voltageRequest.withOutput(Volts.of(0)));
    }

    @Override
    public void stopUptake() {
        uptakeMotor.setControl(voltageRequest.withOutput(Volts.of(0)));
    }

    @Override
    public void zeroRotationPosition(double rotations) {
        rotationMotor.setPosition(Rotations.of(rotations));
    }
}
