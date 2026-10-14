package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.LimelightHelpers;
import frc.robot.constants.subsystems.VisionConstants;
import edu.wpi.first.math.util.Units;
import java.util.function.DoubleSupplier;

public class LimelightCamera extends VisionCamera {

    // supplier for robot field-centric heading
    private final DoubleSupplier yawDegrees;

    // timestamp that tags were last seen
    private double lastTimestamp;

    // previous heartbeat of limelight to record whether camera is connected
    private double lastHeartbeat = -1;

    // counter to check camera connection every 10 periodics
    private int loopCounter = 0;

    // state of connection of the limelight
    private boolean isConnected = false;

    /**
     * Constructor
     * 
     * @param name   name of the camera
     * @param offset robot-to-camera offset
     * @param layout AprilTag field layout
     * @param getYaw robot field-centric heading in degrees
     */

    public LimelightCamera(String name, Transform3d offset, AprilTagFieldLayout layout, DoubleSupplier yawDegrees,
            double trustFactor) {
        super(name, offset, layout, trustFactor);
        this.yawDegrees = yawDegrees;
        Transform3d o = offset;
        LimelightHelpers.setCameraPose_RobotSpace(
                name,
                o.getX(), o.getY(), o.getZ(), // meters: forward, side, up
                Units.radiansToDegrees(o.getRotation().getX()), // roll (degrees)
                Units.radiansToDegrees(o.getRotation().getY()), // pitch (degrees)
                Units.radiansToDegrees(o.getRotation().getZ())); // yaw (degrees)
    }

    /**
     * Constructor
     * 
     * @param name   name of the camera
     * @param offset robot-to-camera offset
     * @param getYaw robot field-centric heading
     */
    public LimelightCamera(String name, Transform3d offset, DoubleSupplier yawDegrees, double trustFactor) {
        this(name, offset, VisionConstants.TAG_LAYOUT, yawDegrees, trustFactor);
        Transform3d o = offset;
        LimelightHelpers.setCameraPose_RobotSpace(
                name,
                o.getX(), o.getY(), o.getZ(), // meters: forward, side, up
                Units.radiansToDegrees(o.getRotation().getX()), // roll (degrees)
                Units.radiansToDegrees(o.getRotation().getY()), // pitch (degrees)
                Units.radiansToDegrees(o.getRotation().getZ())); // yaw (degrees)

    }

    /**
     * Updates pose estimations and camera connected state. This method should be
     * called every periodic.
     * 
     * @return a list of new PoseObservation instances calculated
     */
    @Override
    public List<PoseObservation> update() {
        double currentHeartbeat = LimelightHelpers.getHeartbeat(super.getName());
        if (loopCounter++ >= 10) {
            isConnected = (currentHeartbeat != lastHeartbeat);
            lastHeartbeat = currentHeartbeat;
            loopCounter = 0;
        }

        List<PoseObservation> poseObservations = new ArrayList<>();
        LimelightHelpers.SetRobotOrientation(super.getName(), yawDegrees.getAsDouble(), 0, 0, 0, 0, 0);
        var poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(super.getName());
        if (poseEstimate == null || poseEstimate.tagCount == 0 || poseEstimate.timestampSeconds == lastTimestamp) {
            return poseObservations;
        }
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

        return poseObservations;
    }

    /**
     * Checks whether the camera is connected to the robot.
     * 
     * @return true if camera is connected, false if disconnected
     */
    @Override
    public boolean isConnected() {
        return isConnected;
    }
}