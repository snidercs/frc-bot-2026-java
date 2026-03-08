// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Turret;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and trigger mappings) should be declared here.
 *
 * Ported from container.hpp/cpp in the C++ project.
 */
public class RobotContainer {

    // ── Swerve drive requests ────────────────────────────────────────────────
    private final double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private final double maxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric()
        .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    // ── Telemetry ────────────────────────────────────────────────────────────
    private final Telemetry logger = new Telemetry(TunerConstants.kSpeedAt12Volts);

    // ── Subsystems ───────────────────────────────────────────────────────────
    private final CommandSwerveDrivetrain drivetrain;
    private final Intake intake = new Intake();
    private final Climber climber = new Climber();
    private final Turret turret = new Turret();
    private final VisionMulti vision = new VisionMulti();

    // ── Auto chooser ─────────────────────────────────────────────────────────
    private SendableChooser<Command> autoChooser = null;

    public RobotContainer() {
        // Construct drivetrain with 250 Hz odometry update frequency
        drivetrain = new CommandSwerveDrivetrain(
            TunerConstants.DrivetrainConstants,
            250,
            TunerConstants.FrontLeft,
            TunerConstants.FrontRight,
            TunerConstants.BackLeft,
            TunerConstants.BackRight
        );

        // Configure PathPlanner AutoBuilder for autonomous
        boolean pathPlannerConfigured = true;
        try {
            drivetrain.configurePathPlanner();
            System.out.println("Successfully configured PathPlanner AutoBuilder");
        } catch (Exception e) {
            pathPlannerConfigured = false;
            System.err.println("Failed to configure PathPlanner: " + e.getMessage());
        }

        if (pathPlannerConfigured) {
            try {
                NamedCommands.registerCommand("shooterOn",     turret.shooterOnCommand());
                NamedCommands.registerCommand("shooterOff",    turret.shooterOffCommand());
                NamedCommands.registerCommand("turretStop",    turret.stopCommand());
                NamedCommands.registerCommand("intakeStart",   intake.startCommand());
                NamedCommands.registerCommand("intakeStutter", intake.stutterCommand());
                NamedCommands.registerCommand("intakeStop",    intake.stopCommand());
                NamedCommands.registerCommand("driveJitter",   jitterCommand());
                NamedCommands.registerCommand("aimAtHub",
                    turret.aimAtHubCommand(() -> drivetrain.getState().Pose));
                NamedCommands.registerCommand("aimAndShootHub",
                    turret.aimAndShootHubCommand(() -> drivetrain.getState().Pose));
                NamedCommands.registerCommand("aimPass",
                    turret.aimPassCommand(() -> drivetrain.getState().Pose));
                NamedCommands.registerCommand("aimAndPass",
                    turret.aimAndPassCommand(() -> drivetrain.getState().Pose));

                autoChooser = AutoBuilder.buildAutoChooser(Config.AUTO_DEFAULT_NAME);
                SmartDashboard.putData("AutoChooser", autoChooser);
            } catch (Exception e) {
                pathPlannerConfigured = false;
                System.err.println("Autobuilder failed: " + e.getMessage());
            }
        }

        // Configure controller bindings based on config
        if (Config.GAMEPAD) {
            configureGamepadBindings();
        } else {
            configureJoystickBindings();
        }

        // Idle while the robot is disabled
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> new SwerveRequest.Idle())
                .ignoringDisable(true)
        );

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    // ── Accessors ────────────────────────────────────────────────────────────

    public CommandSwerveDrivetrain getDrivetrain() { return drivetrain; }
    public Intake getIntake() { return intake; }
    public Climber getClimber() { return climber; }
    public Turret getTurret() { return turret; }
    public VisionMulti getVision() { return vision; }

    // ── Autonomous ───────────────────────────────────────────────────────────

    public Command getAutonomousCommand() {
        String autoName = Config.AUTO_DEFAULT_NAME;
        if (autoChooser != null) {
            Command selected = autoChooser.getSelected();
            if (selected != null) {
                autoName = selected.getName();
            }
        }

        try {
            return new PathPlannerAuto(autoName);
        } catch (Exception e) {
            System.err.println("Failed to load PathPlanner auto '" + autoName + "': " + e.getMessage());
            return Commands.none();
        }
    }

    // ── Jitter command ───────────────────────────────────────────────────────

    /**
     * Wiggles the robot back and forth (±2 in) to help seat game pieces in the intake.
     *
     * @param leftToRight If true, jitters left/right (Y axis); otherwise front/back (X axis).
     */
    public Command jitterCommand(boolean leftToRight) {
        final double kJitterSpeed = 0.3; // m/s
        final double kJitterDuration = 0.17; // seconds

        SwerveRequest neg = leftToRight
            ? new SwerveRequest.RobotCentric().withVelocityY(-kJitterSpeed)
            : new SwerveRequest.RobotCentric().withVelocityX(-kJitterSpeed);
        SwerveRequest pos = leftToRight
            ? new SwerveRequest.RobotCentric().withVelocityY(kJitterSpeed)
            : new SwerveRequest.RobotCentric().withVelocityX(kJitterSpeed);

        return Commands.sequence(
            drivetrain.applyRequest(() -> neg).withTimeout(kJitterDuration),
            drivetrain.applyRequest(() -> pos).withTimeout(kJitterDuration),
            drivetrain.applyRequest(() -> neg).withTimeout(kJitterDuration),
            drivetrain.applyRequest(() -> pos).withTimeout(kJitterDuration),
            drivetrain.applyRequest(SwerveRequest.SwerveDriveBrake::new).withTimeout(0.1)
        ).withName("DriveJitter");
    }

    /** Default jitter command (front/back). */
    public Command jitterCommand() {
        return jitterCommand(false);
    }

    // ── Joystick bindings ────────────────────────────────────────────────────

    private void configureJoystickBindings() {
        CommandJoystick stick0 = new CommandJoystick(0);
        CommandJoystick stick1 = new CommandJoystick(1);

        final double deadband = Config.DRIVE_DEADBAND;
        final double exponent = Config.DRIVE_INPUT_EXPONENT;
        final double rotExp = Config.ROTATE_INPUT_EXPONENT;
        final double rotDead = Config.ROTATE_DEADBAND;

        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> drive
                .withVelocityX(-InputUtil.applyCurve(
                    stick0.getHID().getRawAxis(1), deadband, exponent) * maxSpeed)
                .withVelocityY(-InputUtil.applyCurve(
                    stick0.getHID().getRawAxis(0), deadband, exponent) * maxSpeed)
                .withRotationalRate(-InputUtil.applyCurve(
                    stick1.getHID().getRawAxis(0), rotDead, rotExp) * maxAngularRate)
            )
        );

        stick0.button(Config.HEADING_BUTTON_INDEX).onTrue(
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric())
        );

        // Intake control
        stick0.button(Config.INTAKE_TRIGGER_INDEX).whileTrue(intake.intakeCommand());
        stick1.button(Config.INTAKE_EJECT_INDEX).whileTrue(intake.ejectCommand());

        // Shooter Control
        stick1.button(Config.TURRET_SHOOT_BUTTON_INDEX).whileTrue(turret.shootCommand());

        // Aim at Hub — hold to continuously track the Hub and spin up
        stick1.button(Config.TURRET_AIM_HUB_BUTTON_INDEX).whileTrue(
            turret.aimAtHubCommand(() -> drivetrain.getState().Pose)
        );

        // Aim and fire at Hub — hold to aim, auto-fires when ready
        stick1.button(Config.TURRET_FIRE_BUTTON_INDEX).whileTrue(
            turret.aimAndShootHubCommand(() -> drivetrain.getState().Pose)
        );

        // Pass mode — hold to aim at the tower and lob fuel at pass speed
        stick1.button(Config.TURRET_AIM_PASS_BUTTON_INDEX).whileTrue(
            turret.aimAndPassCommand(() -> drivetrain.getState().Pose)
        );

        // Climber control
        stick0.button(Config.CLIMBER_CLIMB_BUTTON_INDEX).whileTrue(climber.climbCommand());
        stick0.button(Config.CLIMBER_LOWER_BUTTON_INDEX).whileTrue(climber.lowerCommand());

        // Drive jitter
        stick0.button(5).onTrue(jitterCommand(false));
        stick0.button(6).onTrue(jitterCommand(true));

        // Disable climber soft limits while held
        stick1.button(4).onTrue(climber.disableSoftLimitsCommand())
                        .onFalse(climber.enableSoftLimitsAndResetCommand());

        // Zero turret rotation position
        stick1.button(3).onTrue(turret.calibrateRotationZero());

        // Manual turret rotation
        final int rotStick = Config.TURRET_ROTATION_AXIS_STICK;
        final int rotIdx = Config.TURRET_ROTATION_AXIS_INDEX;
        final double rotGain = Config.TURRET_ROTATION_GAIN;
        CommandJoystick[] sticks = {stick0, stick1};
        turret.setDefaultCommand(
            turret.manualRotateCommand(() ->
                rotGain * sticks[rotStick].getHID().getRawAxis(rotIdx)
            )
        );
    }

    // ── Gamepad bindings ─────────────────────────────────────────────────────

    private void configureGamepadBindings() {
        CommandXboxController joystick = new CommandXboxController(0);

        final double deadband = Config.DRIVE_DEADBAND;
        final double exponent = Config.DRIVE_INPUT_EXPONENT;

        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> drive
                .withVelocityX(-InputUtil.applyCurve(joystick.getLeftY(), deadband, exponent) * maxSpeed)
                .withVelocityY(-InputUtil.applyCurve(joystick.getLeftX(), deadband, exponent) * maxSpeed)
                .withRotationalRate(-InputUtil.applyCurve(joystick.getRightX(), deadband, exponent) * maxAngularRate)
            )
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kReverse));

        // Reset field-centric heading
        joystick.leftBumper().onTrue(
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric())
        );

        // Intake controls
        joystick.rightBumper().whileTrue(intake.intakeCommand());
        joystick.rightTrigger().whileTrue(intake.ejectCommand());

        // Climber controls
        joystick.button(Config.CLIMBER_CLIMB_BUTTON_INDEX).whileTrue(climber.climbCommand());
        joystick.button(Config.CLIMBER_LOWER_BUTTON_INDEX).whileTrue(climber.lowerCommand());
    }
}
