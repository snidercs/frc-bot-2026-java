// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

/** Hardware abstraction for the Climber subsystem. */
public interface ClimberIO {

    @AutoLog
    public static class ClimberIOInputs {
        public double velocityRps = 0;
        public double positionRotations = 0;
        public double currentAmps = 0;
        public double voltage = 0;
    }

    /** Read hardware sensors into the inputs struct. */
    default void updateInputs(ClimberIOInputs inputs) {}

    /** Set motor duty cycle output (-1 to 1). */
    default void setDutyCycle(double dutyCycle) {}

    /** Stop the motor. */
    default void stop() {}

    /** Disable forward and reverse software limits. */
    default void disableSoftLimits() {}

    /** Re-zero the encoder and re-enable software limits. */
    default void enableSoftLimitsAndResetPosition(double forwardLimit, double reverseLimit) {}
}
