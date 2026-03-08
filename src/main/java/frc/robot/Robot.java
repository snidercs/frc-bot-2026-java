// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 *
 * Ported from robot.hpp/cpp in the C++ project.
 */
public class Robot extends LoggedRobot {
    private Command autoCommand;
    private RobotContainer container;

    /**
     * Enable vision processing. Set to true to enable vision measurements
     * in RobotPeriodic.
     */
    private static final boolean BOT_VISION = true;

    // NOTE: Changes to this constructor or robotInit() should be reflected in
    //       README.md → "Power-Up Initialization".
    public Robot() {
        // AdvantageKit logger configuration — must run before any other init
        Logger.recordMetadata("ProjectName", "frc-bot-2026-java");

        if (isReal()) {
            Logger.addDataReceiver(new WPILOGWriter()); // Log to USB stick ("/U/logs")
            Logger.addDataReceiver(new NT4Publisher());  // Publish to NetworkTables
        } else {
            setUseTiming(false); // Run as fast as possible in replay
            String logPath = LogFileUtil.findReplayLog();
            Logger.setReplaySource(new WPILOGReader(logPath));
            Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        }

        Logger.start();
    }

    @Override
    public void robotInit() {
        // Use 6V brownout threshold (same as C++ project)
        RobotController.setBrownoutVoltage(6.0);

        // Silence joystick warnings during init
        DriverStation.silenceJoystickConnectionWarning(true);

        container = new RobotContainer();

        SmartDashboard.putString("Controller", Config.GAMEPAD ? "Gamepad" : "Flightsticks");
        Config.display();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        if (BOT_VISION) {
            // Poll all cameras and fuse measurements into the drivetrain pose estimator
            for (Vision.VisionMeasurement measurement : container.getVision().getMeasurements()) {
                double[] sd = measurement.stdDevs();
                container.getDrivetrain().addVisionMeasurement(
                    measurement.pose(),
                    measurement.timestampSeconds(),
                    VecBuilder.fill(sd[0], sd[1], sd[2])
                );
            }

            // Estimated distance from fused robot pose to the hub
            var robotPose = container.getDrivetrain().getState().Pose;
            double distanceToHub = robotPose.getTranslation().getDistance(Vision.hubPosition());
            SmartDashboard.putNumber("Robot/DistanceToHub (m)", distanceToHub);
        }
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        System.out.println("AutonomousInit: Getting autonomous command...");
        autoCommand = container.getAutonomousCommand();

        if (autoCommand != null) {
            System.out.println("AutonomousInit: Scheduling autonomous command");
            CommandScheduler.getInstance().schedule(autoCommand);
        } else {
            System.err.println("AutonomousInit: No autonomous command returned!");
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {
        autoCommand = null;
    }

    @Override
    public void teleopInit() {
        DriverStation.silenceJoystickConnectionWarning(false);

        // Make sure autonomous stops running when teleop starts
        if (autoCommand != null) {
            autoCommand.cancel();
            autoCommand = null;
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {
        DriverStation.silenceJoystickConnectionWarning(true);
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationInit() {}

    @Override
    public void simulationPeriodic() {}
}
