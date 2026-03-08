// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

import org.littletonrobotics.junction.Logger;

/**
 * Swerve drive telemetry — publishes drivetrain state to NetworkTables
 * and SmartDashboard Mechanism2d widgets.
 *
 * Ported from telemetry.hpp/cpp in the C++ project.
 */
public class Telemetry {
    private final double maxSpeed;

    /* Mechanisms to represent the swerve module states */
    private final Mechanism2d[] moduleMechanisms = {
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
    };

    /* A direction and length changing ligament for speed representation */
    private final MechanismLigament2d[] moduleSpeeds = {
        moduleMechanisms[0].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        moduleMechanisms[1].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        moduleMechanisms[2].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        moduleMechanisms[3].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
    };

    /* A direction changing and length constant ligament for module direction */
    private final MechanismLigament2d[] moduleDirections = {
        moduleMechanisms[0].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        moduleMechanisms[1].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        moduleMechanisms[2].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        moduleMechanisms[3].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
    };

    /**
     * Construct a telemetry object with the specified max speed of the robot.
     *
     * @param maxSpeed Maximum speed in meters per second
     */
    public Telemetry(LinearVelocity maxSpeed) {
        this.maxSpeed = maxSpeed.in(edu.wpi.first.units.Units.MetersPerSecond);

        /* Set up the module state Mechanism2d telemetry */
        for (int i = 0; i < moduleSpeeds.length; i++) {
            SmartDashboard.putData("Module " + i, moduleMechanisms[i]);
        }
    }

    /** Accept the swerve drive state and telemeterize it to AdvantageKit and SmartDashboard. */
    public void telemeterize(SwerveDriveState state) {
        /* Telemeterize the swerve drive state */
        Logger.recordOutput("DriveState/Pose", state.Pose);
        Logger.recordOutput("DriveState/Speeds", state.Speeds);
        Logger.recordOutput("DriveState/ModuleStates", state.ModuleStates);
        Logger.recordOutput("DriveState/ModuleTargets", state.ModuleTargets);
        Logger.recordOutput("DriveState/ModulePositions", state.ModulePositions);
        Logger.recordOutput("DriveState/Timestamp", state.Timestamp);
        Logger.recordOutput("DriveState/OdometryFrequency", 1.0 / state.OdometryPeriod);

        /* Telemeterize the pose to a Field2d */
        Logger.recordOutput("Pose/robotPose", state.Pose);

        /* Telemeterize each module state to a Mechanism2d */
        for (int i = 0; i < moduleSpeeds.length; i++) {
            moduleDirections[i].setAngle(state.ModuleStates[i].angle.getDegrees());
            moduleSpeeds[i].setAngle(state.ModuleStates[i].angle.getDegrees());
            moduleSpeeds[i].setLength(state.ModuleStates[i].speedMetersPerSecond / (2 * maxSpeed));
        }
    }
}
