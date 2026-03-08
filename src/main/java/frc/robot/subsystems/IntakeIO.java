// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

/** Hardware abstraction for the Intake subsystem. */
public interface IntakeIO {

    @AutoLog
    public static class IntakeIOInputs {
        public double topVelocityRps = 0;
        public double bottomVelocityRps = 0;
        public double topCurrentAmps = 0;
        public double bottomCurrentAmps = 0;
    }

    /** Read hardware sensors into the inputs struct. */
    default void updateInputs(IntakeIOInputs inputs) {}

    /** Set voltage output for both motors. */
    default void setVoltage(double voltage) {}

    /** Stop both motors. */
    default void stop() {}
}
