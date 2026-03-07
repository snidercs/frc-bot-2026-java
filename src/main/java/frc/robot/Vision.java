// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

/**
 * Vision system configuration constants and data types.
 *
 * Contains fixed configuration parameters for camera mounting positions,
 * turret geometry, and AprilTag field information.
 *
 * Ported from vision.hpp in the C++ project.
 */
public final class Vision {
    private Vision() {}

    // ── Data types ───────────────────────────────────────────────────────────

    /**
     * A single vision pose measurement from a camera.
     */
    public record VisionMeasurement(
        /** The estimated robot pose on the field */
        Pose2d pose,
        /** The timestamp when this measurement was captured (seconds) */
        double timestampSeconds,
        /** Standard deviations for pose uncertainty [x, y, theta] */
        double[] stdDevs,
        /** Source camera name (FL, FR, BL, BR) */
        String source
    ) {}

    /**
     * Interface for vision measurement sources (real hardware or simulation).
     */
    public interface VisionIO {
        /**
         * Retrieves all available vision measurements from the last update cycle.
         *
         * <p><b>Warning:</b> Must be called <i>exactly once per periodic cycle</i>. Implementations
         * clear and refill an internal buffer on each call — calling it more than once
         * in the same cycle will return an empty list on the second call.
         *
         * @return List of measurements; valid until the next call.
         */
        List<VisionMeasurement> getMeasurements();

        /** Gets a human-readable status string for debugging. */
        default String getStatus() { return "VisionIO base class"; }

        /** Gets the most recent target information from all cameras. */
        default String getLastTargets() { return "No targets"; }

        /** Gets counts of rejected measurements for debugging. */
        default String getRejectedCounts() { return "No rejection data"; }
    }

    // ── Camera names ─────────────────────────────────────────────────────────
    /** Camera names (must match PhotonVision configuration) */
    public static final String[] CAMERA_NAMES = {"FL", "FR", "BL", "BR"};

    // ── Camera transforms ────────────────────────────────────────────────────
    /**
     * Camera mounting positions relative to robot center.
     * Index order: FL=0, FR=1, BL=2, BR=3
     * Cameras are fixed to chassis (not turret), angled ~45° outward.
     *
     * TODO: FR and BR cameras are not yet mounted — transforms are placeholders.
     */
    public static final Transform3d[] ROBOT_TO_CAMERA = {
        // Front-Left camera: on front edge, 6cm from left side, 51cm high
        new Transform3d(
            new Translation3d(Units.inchesToMeters(13.74), Units.inchesToMeters(11.5), Units.inchesToMeters(15.0)),
            new Rotation3d(0, 0, 0)
        ),
        // Front-Right camera (not yet mounted — placeholder)
        new Transform3d(
            new Translation3d(Units.inchesToMeters(9.84), Units.inchesToMeters(-9.84), Units.inchesToMeters(19.69)),
            new Rotation3d(0, 0, Math.toRadians(-45))
        ),
        // Back-Left camera: 3.5cm past back edge, 37cm from right side, 41cm high
        new Transform3d(
            new Translation3d(Units.inchesToMeters(-15.00), Units.inchesToMeters(-12.25), Units.inchesToMeters(11.35)),
            new Rotation3d(0, 0, Math.toRadians(180))
        ),
        // Back-Right camera
        new Transform3d(
            new Translation3d(Units.inchesToMeters(-9.84), Units.inchesToMeters(-9.84), Units.inchesToMeters(19.69)),
            new Rotation3d(0, 0, Math.toRadians(-135))
        )
    };

    // ── Turret pivot ─────────────────────────────────────────────────────────
    /** Turret pivot point location in robot frame (meters from robot center). */
    public static final Translation2d TURRET_PIVOT_IN_ROBOT = new Translation2d(0.0, 0.0);

    // ── Field layout ─────────────────────────────────────────────────────────
    /**
     * AprilTag field layout for the current season.
     * Returns empty layout if load fails.
     */
    public static AprilTagFieldLayout getFieldLayout() {
        return AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
    }

    /** Valid goal tag IDs. */
    public static final int[] GOAL_TAG_IDS = {1, 2, 3, 4, 5, 6, 7, 8};

    // ── Landmarks ────────────────────────────────────────────────────────────
    /**
     * Returns the hub (goal) position for the current alliance.
     * Defaults to the red alliance position when alliance is unknown.
     */
    public static Translation2d hubPosition() {
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue) {
            return new Translation2d(Units.inchesToMeters(182.105), Units.inchesToMeters(158.845));
        }
        return new Translation2d(Units.inchesToMeters(469.115), Units.inchesToMeters(158.845));
    }
}
