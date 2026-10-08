package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.LimelightHelpers;
import frc.robot.constants.subsystems.VisionConstants;

public class LimelightCamera extends VisionCamera {
    private final Supplier<Double> getYaw;
    private double lastTimestamp;
    private double lastHeartbeat = -1;
    private int loopCounter = 0;
    private boolean isConnected = false;

    public LimelightCamera(String name, Transform3d offset, AprilTagFieldLayout layout, Supplier<Double> getYaw) {
        super(name, offset, layout);
        this.getYaw = getYaw;
    }

    public LimelightCamera(String name, Transform3d offset, Supplier<Double> getYaw) {
        this(name, offset, VisionConstants.TAG_LAYOUT, getYaw);
    }

    @Override
    public List<PoseObservation> update() {
        List<PoseObservation> poseObservations = new ArrayList<>();
        LimelightHelpers.SetRobotOrientation(super.getName(), getYaw.get(), 0, 0, 0, 0, 0);
        var poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(super.getName());
        if (poseEstimate.timestampSeconds != lastTimestamp) {
            lastTimestamp = poseEstimate.timestampSeconds;
            if (poseEstimate != null && poseEstimate.tagCount > 0) {
                poseObservations.add(new PoseObservation(
                        poseEstimate.pose,
                        poseEstimate.timestampSeconds,
                        poseEstimate.tagCount,
                        poseEstimate.avgTagDist,
                        poseEstimate.rawFiducials.length > 0 ? poseEstimate.rawFiducials[0].ambiguity : 0.0,
                        PoseType.MEGA_TAG_2));
            }
        }

        double currentHeartbeat = LimelightHelpers.getHeartbeat(super.getName());
        if (loopCounter++ >= 10) {
            isConnected = (currentHeartbeat != lastHeartbeat);
            lastHeartbeat = currentHeartbeat;
            loopCounter = 0;
        }

        return poseObservations;
    }

    @Override
    public boolean isConnected() {
        return isConnected;
    }
}