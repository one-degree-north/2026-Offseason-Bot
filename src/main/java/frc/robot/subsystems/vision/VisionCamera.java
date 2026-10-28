package frc.robot.subsystems.vision;

import java.util.List;

import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;

public abstract class VisionCamera {
    // name of the camera
    private final String name;

    // robot-to-camera offset of the camera
    private final Transform3d offset;

    // the AprilTag layout on the field
    private final AprilTagFieldLayout fieldLayout;

    /**
     * PoseType enum
     */
    public enum PoseType {
        /**
         * PhotonVision single tag
         */
        PHOTON_SINGLE_TAG,

        /**
         * PhotonVision multi tag
         */
        PHOTON_MULTI_TAG,

        /**
         * MegaTag2 (Limelight)
         */
        MEGA_TAG_2
    }

    private final double trustFactor;

    /** Multiplier on this camera's std devs. 1.0 = normal; larger = trust less. */

    /**
     * PoseObservation record
     * 
     * @param pose               the estimated pose
     * @param timestamp          the FPGA timestamp
     * @param tagCount           the number of tags in view
     * @param averageTagDistance the average distance in meters
     * @param ambiguity          the ambiguity level for single-tag, in the range
     *                           [0, 1], higher means more ambiguous
     * @param type               the type of pose estimation
     */
    public record PoseObservation(
            Pose2d pose, double timestamp, int tagCount, double averageTagDistance, double ambiguity, PoseType type) {
    }

    /**
     * Constructor
     * 
     * @param name        name of the camera
     * @param offset      robot-to-camera offset
     * @param layout      AprilTag field layout
     * @param trustFactor
     */
    public VisionCamera(String name, Transform3d offset, AprilTagFieldLayout layout, double trustFactor) {
        this.name = name;
        this.offset = offset;
        this.fieldLayout = layout;
        this.trustFactor = trustFactor;
    }

    public double getTrustFactor() {
        return trustFactor;
    }

    /**
     * Get the camera name
     * 
     * @return the camera name
     */
    public String getName() {
        return name;
    }

    /**
     * Get the camera robot-to-camera offset
     * 
     * @return the robot-to-camera offset
     */
    public Transform3d getOffset() {
        return offset;
    }

    /**
     * Get the field AprilTag layout
     * 
     * @return the field AprilTag layout
     */
    public AprilTagFieldLayout getFieldLayout() {
        return fieldLayout;
    }

    /**
     * Update the camera pose estimation
     * 
     * @return a list of pose observations since the last update
     */
    public abstract List<PoseObservation> update();

    /**
     * Whether the camera can be reached and is active
     * 
     * @return true if the camera is connected, false otherwise
     */
    public abstract boolean isConnected();

    public void addToSimulation(VisionSystemSim sim) {}
}

