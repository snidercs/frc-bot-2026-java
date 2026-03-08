// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

/** Hardware abstraction for the Turret subsystem. */
public interface TurretIO {

    @AutoLog
    public static class TurretIOInputs {
        public double rotationPositionRotations = 0;
        public double rotationVelocityRps = 0;
        public double rotationVoltage = 0;
        public double rotationCurrentAmps = 0;
        public double shooterVelocityRps = 0;
    }

    /** Read hardware sensors into the inputs struct. */
    default void updateInputs(TurretIOInputs inputs) {}

    /** Command rotation motor to a position in rotations. */
    default void setRotationPosition(double rotations) {}

    /** Command rotation motor to a velocity in rotations per second. */
    default void setRotationVelocity(double rps) {}

    /** Command rotation motor to a duty cycle output (-1 to 1). */
    default void setRotationDutyCycle(double dutyCycle) {}

    /** Command shooter flywheel to a velocity in rotations per second. */
    default void setShooterVelocity(double rps) {}

    /** Command uptake feeder to a velocity in rotations per second. */
    default void setUptakeVelocity(double rps) {}

    /** Stop the rotation motor. */
    default void stopRotation() {}

    /** Stop the shooter flywheel. */
    default void stopShooter() {}

    /** Stop the uptake feeder. */
    default void stopUptake() {}

    /** Zero the rotation encoder to the given position in rotations. */
    default void zeroRotationPosition(double rotations) {}
}
