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
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Config;

/**
 * Intake subsystem with two Kraken X44 motors (top and bottom).
 *
 * Ported from intake.hpp/cpp in the C++ project.
 */
public class Intake extends SubsystemBase {

    // Two Kraken x44 motors
    private final TalonFX m_topMotor = new TalonFX(Config.INTAKE_TOP_DEVICE_ID, new CANBus(Config.INTAKE_TOP_CAN_BUS));
    private final TalonFX m_bottomMotor = new TalonFX(Config.INTAKE_BOTTOM_DEVICE_ID, new CANBus(Config.INTAKE_BOTTOM_CAN_BUS));

    // Control requests (reusable)
    private final VoltageOut m_voltageRequest = new VoltageOut(0);
    private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0);

    // Constants
    private final double kIntakeVoltage = Config.INTAKE_VOLTAGE;
    private static final double kEjectVoltage = 2.0;

    public Intake() {
        setName("Intake");
        configureMotors();
    }

    private void configureMotors() {
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
        m_topMotor.getConfigurator().apply(topConfig);
        m_bottomMotor.getConfigurator().apply(bottomConfig);

        // Configure status signal update frequencies to prevent CAN stale errors
        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            m_topMotor.getVelocity(),
            m_topMotor.getSupplyCurrent(),
            m_bottomMotor.getVelocity(),
            m_bottomMotor.getSupplyCurrent()
        );

        // Optimize CAN bus utilization
        m_topMotor.optimizeBusUtilization();
        m_bottomMotor.optimizeBusUtilization();
    }

    @Override
    public void periodic() {
        // Telemetry for debugging and monitoring (disabled by default)
        // SmartDashboard.putNumber("Intake/Top Velocity (rps)", m_topMotor.getVelocity().getValue().in(RotationsPerSecond));
        // SmartDashboard.putNumber("Intake/Bottom Velocity (rps)", m_bottomMotor.getVelocity().getValue().in(RotationsPerSecond));
    }

    // Manual control
    public void setVoltage(double voltage) {
        m_topMotor.setControl(m_voltageRequest.withOutput(Volts.of(voltage)));
        m_bottomMotor.setControl(m_voltageRequest.withOutput(Volts.of(voltage)));
    }

    public void stop() {
        m_topMotor.setControl(m_voltageRequest.withOutput(Volts.of(0)));
        m_bottomMotor.setControl(m_voltageRequest.withOutput(Volts.of(0)));
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
