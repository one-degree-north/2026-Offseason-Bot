package frc.robot.constants.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
    /**
     * Name of camera 1 (PhotonVision)
     */
    public static final String CAMERA_1_NAME = "4817PhotonVisionCam1";

    /**
     * Name of camera 2 (Limelight)
     */
    public static final String CAMERA_2_NAME = "limelight";

    /**
     * The robot-to-camera offset of camera 1
     * Cam mounted facing forward, half a meter forward of center, half a meter up
     * from center.
     */
    public static final Transform3d CAM_1_OFFSET = new Transform3d(new Translation3d(0.5, 0.0, 0.5),
            new Rotation3d(0, 0, 0));

    /**
     * The robot-to-camera offset of camera 2
     * Cam mounted facing backward, half a meter backward of center, half a meter up
     * from center.
     */
    public static final Transform3d CAM_2_OFFSET = new Transform3d(new Translation3d(-0.5, 0.0, 0.5),
            new Rotation3d(0, 0, Units.degreesToRadians(180)));

    /**
     * The AprilTag layout on the field
     */
    public static final AprilTagFieldLayout TAG_LAYOUT = AprilTagFieldLayout
            .loadField(AprilTagFields.kDefaultField);

    /**
     * Maximum ambiguity we will take for a AprilTag pose estimation
     */
    public static final double MAX_AMBIGUITY = 0.2;
}