// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

/**
 * Multi-camera vision implementation using all four chassis-mounted cameras.
 *
 * Each camera has its own PhotonPoseEstimator seeded with the camera's
 * robot-relative transform. All cameras are polled every cycle; results that
 * pass gating (latency, ambiguity) are returned as individual VisionMeasurement
 * entries for the pose estimator to fuse via AddVisionMeasurement().
 *
 * Ported from visionmulti.hpp/cpp in the C++ project.
 */
public class VisionMulti implements Vision.VisionIO {

    private static final double MAX_AMBIGUITY = 0.3;
    private static final double MAX_LATENCY_SECONDS = 0.5;

    /**
     * One camera + estimator pair per physical camera.
     */
    private static class CameraUnit {
        final PhotonCamera camera;
        final PhotonPoseEstimator estimator;

        CameraUnit(String name, Transform3d robotToCamera, AprilTagFieldLayout layout) {
            this.camera = new PhotonCamera(name);
            this.estimator = new PhotonPoseEstimator(
                layout,
                robotToCamera
            );
        }
    }

    private final AprilTagFieldLayout fieldLayout;
    private final CameraUnit[] cameras;

    // Pre-allocated measurement buffer — cleared and refilled each cycle
    private final List<Vision.VisionMeasurement> measurements = new ArrayList<>();

    // Raw camera results cached by getMeasurements()
    @SuppressWarnings("unchecked")
    private final List<PhotonPipelineResult>[] rawResults = new List[4];

    // Rejection counters (across all cameras)
    private int rejectedNoTargets = 0;
    private int rejectedStale = 0;
    private int rejectedAmbiguous = 0;
    private int rejectedOutOfBounds = 0;
    private int acceptedCount = 0;

    public VisionMulti() {
        this.fieldLayout = Vision.getFieldLayout();
        this.cameras = new CameraUnit[Vision.CAMERA_NAMES.length];
        for (int i = 0; i < Vision.CAMERA_NAMES.length; i++) {
            cameras[i] = new CameraUnit(
                Vision.CAMERA_NAMES[i],
                Vision.ROBOT_TO_CAMERA[i],
                fieldLayout
            );
            rawResults[i] = new ArrayList<>();
        }
    }

    @Override
    public List<Vision.VisionMeasurement> getMeasurements() {
        measurements.clear();

        for (int i = 0; i < cameras.length; i++) {
            CameraUnit unit = cameras[i];
            rawResults[i] = unit.camera.getAllUnreadResults();

            for (PhotonPipelineResult result : rawResults[i]) {
                if (!result.hasTargets()) {
                    rejectedNoTargets++;
                    continue;
                }

                // Gate on latency
                double latencySeconds = result.metadata.getLatencyMillis() / 1000.0;
                if (latencySeconds > MAX_LATENCY_SECONDS) {
                    rejectedStale++;
                    continue;
                }

                // Gate on best-target ambiguity
                if (result.getBestTarget().getPoseAmbiguity() > MAX_AMBIGUITY) {
                    rejectedAmbiguous++;
                    continue;
                }

                // Let PhotonPoseEstimator do the heavy lifting
                var estimatedPose = unit.estimator.estimateLowestAmbiguityPose(result);
                if (estimatedPose.isEmpty()) {
                    rejectedNoTargets++;
                    continue;
                }

                // Compute rough distance to best tag for std dev scaling
                var bestTransform = result.getBestTarget().getBestCameraToTarget();
                double distance = Math.sqrt(
                    bestTransform.getX() * bestTransform.getX() +
                    bestTransform.getY() * bestTransform.getY() +
                    bestTransform.getZ() * bestTransform.getZ()
                );

                // Sanity gate: reject poses outside field boundaries (16.46m x 8.21m)
                Pose2d pose2d = estimatedPose.get().estimatedPose.toPose2d();
                if (pose2d.getX() < 0 || pose2d.getX() > 16.46 ||
                    pose2d.getY() < 0 || pose2d.getY() > 8.21) {
                    rejectedOutOfBounds++;
                    continue;
                }

                measurements.add(new Vision.VisionMeasurement(
                    pose2d,
                    estimatedPose.get().timestampSeconds,
                    computeStdDevs(distance),
                    unit.camera.getName()
                ));

                // Telemetry: log each accepted pose
                String camName = unit.camera.getName();
                SmartDashboard.putNumber("Vision/" + camName + "/X (m)", pose2d.getX());
                SmartDashboard.putNumber("Vision/" + camName + "/Y (m)", pose2d.getY());
                SmartDashboard.putNumber("Vision/" + camName + "/Rot (deg)", pose2d.getRotation().getDegrees());

                acceptedCount++;
            }
        }

        SmartDashboard.putNumber("Vision/Accepted", acceptedCount);
        SmartDashboard.putNumber("Vision/Rejected OutOfBounds", rejectedOutOfBounds);

        return measurements;
    }

    private double[] computeStdDevs(double distanceMeters) {
        double xy = 0.01 + (distanceMeters * 0.05);    // 1 cm base + 5 cm/m
        double theta = 0.01 + (distanceMeters * 0.02);  // 0.01 rad base + 0.02/m
        return new double[]{xy, xy, theta};
    }

    @Override
    public String getStatus() {
        int activeCameras = 0;
        for (CameraUnit unit : cameras) {
            if (unit.camera.isConnected()) {
                activeCameras++;
            }
        }
        return "VisionMulti - " + activeCameras + "/4 cameras connected";
    }

    @Override
    public String getLastTargets() {
        StringBuilder info = new StringBuilder();
        for (int i = 0; i < cameras.length; i++) {
            List<PhotonPipelineResult> results = rawResults[i];
            if (results.isEmpty() || !results.get(results.size() - 1).hasTargets()) {
                continue;
            }
            info.append("[").append(cameras[i].camera.getName()).append("] ");
            for (var target : results.get(results.size() - 1).getTargets()) {
                info.append("ID=").append(target.getFiducialId()).append(" ");
            }
        }
        return info.isEmpty() ? "No targets" : info.toString();
    }

    @Override
    public String getRejectedCounts() {
        return "Accepted: " + acceptedCount
             + " | Rejected: NoTargets=" + rejectedNoTargets
             + " Stale=" + rejectedStale
             + " Ambiguous=" + rejectedAmbiguous
             + " OutOfBounds=" + rejectedOutOfBounds;
    }
}
