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

        /** Single-tag poses farther than this are too inaccurate to use (meters). */
        public static final double MAX_SINGLE_TAG_DISTANCE_METERS = 4.0;

        /**
         * Reject MegaTag2 poses while spinning faster than this (rotations per second).
         */
        public static final double MAX_YAW_RATE_ROT_PER_SEC = 2.0;

        // ---------- Trust: standard deviations ----------

        /** x/y std dev for ONE tag seen from ONE meter away (meters). */
        public static final double LINEAR_STD_DEV_BASELINE_METERS = 0.02;

        /** Heading std dev for ONE tag seen from ONE meter away (radians). */
        public static final double ANGULAR_STD_DEV_BASELINE_RADIANS = 0.06;

        /**
         * MegaTag2 position is extra good (it uses our gyro), so shrink its x/y std
         * devs.
         */
        public static final double MEGATAG2_LINEAR_FACTOR = 0.5;

        /** Treat any tag closer than this as this far, so std devs never reach zero. */
        public static final double MIN_TAG_DISTANCE_METERS = 0.5;

        /** A std dev so large the estimator effectively ignores that value. */
        public static final double IGNORE_STD_DEV = 1e6;

        /** Per-camera trust multipliers (1.0 = normal, bigger = trust less). */
        public static final double CAM_1_TRUST_FACTOR = 1.0;
        public static final double CAM_2_TRUST_FACTOR = 1.0;
}