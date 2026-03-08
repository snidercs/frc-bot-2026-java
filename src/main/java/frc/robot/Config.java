// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

/**
 * Robot configuration constants, ported from the Lua config system in the C++ project.
 * All configuration values from robot/config.lua are defined here as static constants.
 */
public final class Config {
    private Config() {}

    // ── General ──────────────────────────────────────────────────────────────
    public static final String ROBOT_NAME = "Indy";
    public static final int TEAM = 9431;
    public static final int PERIOD_MS = 20;

    /** true → single Xbox controller; false → dual flight sticks */
    public static final boolean GAMEPAD = false;

    public static final String AUTO_DEFAULT_NAME = "Backup-Shoot-Left";

    // ── Heading reset ────────────────────────────────────────────────────────
    public static final int HEADING_BUTTON_INDEX = 8;

    // ── Intake ───────────────────────────────────────────────────────────────
    public static final int INTAKE_TRIGGER_INDEX = 18;
    public static final int INTAKE_EJECT_INDEX = 19;
    public static final int INTAKE_TOP_DEVICE_ID = 14;
    public static final String INTAKE_TOP_CAN_BUS = "rio";
    public static final int INTAKE_BOTTOM_DEVICE_ID = 15;
    public static final String INTAKE_BOTTOM_CAN_BUS = "rio";
    /** Voltage for intake motor (negative = intake direction) */
    public static final double INTAKE_VOLTAGE = -4.2;
    /** Seconds for stutter command duration */
    public static final double INTAKE_STUTTER_LENGTH = 9.25;

    // ── Climber ──────────────────────────────────────────────────────────────
    public static final int CLIMBER_DEVICE_ID = 1;
    public static final String CLIMBER_CAN_BUS = "rio";
    public static final int CLIMBER_CLIMB_BUTTON_INDEX = 16;
    public static final int CLIMBER_LOWER_BUTTON_INDEX = 1;

    // ── Turret / Shooter ─────────────────────────────────────────────────────
    public static final int TURRET_ROTATION_DEVICE_ID = 19;
    public static final String TURRET_ROTATION_CAN_BUS = "rio";
    public static final int TURRET_ROTATION_AXIS_STICK = 0;
    public static final int TURRET_ROTATION_AXIS_INDEX = 3;
    public static final double TURRET_ROTATION_GAIN = 0.1;

    public static final int TURRET_SHOOTER_DEVICE_ID = 16;
    public static final String TURRET_SHOOTER_CAN_BUS = "rio";
    public static final int TURRET_AIM_BUTTON_INDEX = 7;
    public static final int TURRET_SHOOT_BUTTON_INDEX = 18;
    public static final int TURRET_AIM_HUB_BUTTON_INDEX = 9;
    public static final int TURRET_AIM_PASS_BUTTON_INDEX = 10;
    public static final int TURRET_FIRE_BUTTON_INDEX = 11;

    public static final int TURRET_UPTAKE_DEVICE_ID = 21;
    public static final String TURRET_UPTAKE_CAN_BUS = "rio";

    // ── Vision ───────────────────────────────────────────────────────────────
    public static final String VISION_TEST_CAMERA = "TestCam";

    // ── Drive input shaping ──────────────────────────────────────────────────
    public static final double DRIVE_DEADBAND = 0.05;
    public static final double DRIVE_INPUT_EXPONENT = 2.5;
    public static final double ROTATE_DEADBAND = 0.05;
    public static final double ROTATE_INPUT_EXPONENT = 1.25;

    /**
     * Displays all configuration to stdout (mirrors the C++ config::display()).
     */
    public static void display() {
        System.out.println("[config] begin settings");
        System.out.println("[config] robot_name = \"" + ROBOT_NAME + "\"");
        System.out.println("[config] team = " + TEAM);
        System.out.println("[config] period = " + PERIOD_MS);
        System.out.println("[config] gamepad = " + GAMEPAD);
        System.out.println("[config] auto_default_name = \"" + AUTO_DEFAULT_NAME + "\"");
        System.out.println("[config] intake_voltage = " + INTAKE_VOLTAGE);
        System.out.println("[config] drive_deadband = " + DRIVE_DEADBAND);
        System.out.println("[config] drive_input_exponent = " + DRIVE_INPUT_EXPONENT);
        System.out.println("[config] rotate_deadband = " + ROTATE_DEADBAND);
        System.out.println("[config] rotate_input_exponent = " + ROTATE_INPUT_EXPONENT);
        System.out.println("[config] end settings");
    }
}
