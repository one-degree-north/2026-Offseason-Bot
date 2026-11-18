package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.IOException;
import java.nio.file.Path;

/**
 * Cone Clash field: the custom AprilTag layout plus the positions of the field
 * elements.
 *
 * <p>
 * Everything here is in the WPILib "blue origin" frame: (0, 0) is the right
 * corner of the blue
 * alliance wall, +X points toward red, +Y points to the blue drivers' left. All
 * values come from
 * generate_field.py, so if you change the field there, update the numbers here
 * too.
 *
 * <p>
 * Put cone_clash.json in src/main/deploy/apriltags/. The deploy folder is
 * copied to the
 * roboRIO on deploy, and in simulation it is read straight from
 * src/main/deploy.
 */
public final class ConeClashField {
    private ConeClashField() {
    }

    public static final double LENGTH_METERS = 16.541;
    public static final double WIDTH_METERS = 8.069;

    /**
     * The custom AprilTag layout. Loaded once, the first time this class is used.
     */
    public static final AprilTagFieldLayout TAG_LAYOUT = loadLayout();

    private static AprilTagFieldLayout loadLayout() {
        Path path = Filesystem.getDeployDirectory().toPath().resolve("apriltags/cone_clash.json");
        try {
            return new AprilTagFieldLayout(path);
        } catch (IOException e) {
            // Fail loudly: a missing layout means vision silently does nothing, which is
            // worse.
            DriverStation.reportError("Could not load Cone Clash AprilTag layout at " + path, e.getStackTrace());
            throw new RuntimeException(e);
        }
    }

    // ---------------------------------------------------------------------------
    // Field elements (blue side). Use AllianceFlip.flip(...) below to get the red
    // version.
    // ---------------------------------------------------------------------------

    /**
     * X of the Level 1 front face of the blue Steps. The front faces the center
     * line (+X).
     */
    public static final double BLUE_STEPS_FRONT_X = 4.50;

    public static final double STEP_TREAD_METERS = Units.inchesToMeters(10);

    /**
     * Heights of Level 1, 2, 3 (manual 2.1). Handy for elevator / arm setpoints.
     */
    public static final double[] STEP_HEIGHTS_METERS = {
            Units.inchesToMeters(24), Units.inchesToMeters(48), Units.inchesToMeters(60)
    };

    /**
     * The three shared center poles (manual 2.2). Index 0 is on the blue drivers'
     * right (low Y),
     * index 2 on their left (high Y), matching POLE_YS in generate_field.py.
     */
    public static final Translation2d[] CENTER_POLES = {
            new Translation2d(LENGTH_METERS / 2, WIDTH_METERS * 0.25),
            new Translation2d(LENGTH_METERS / 2, WIDTH_METERS * 0.50),
            new Translation2d(LENGTH_METERS / 2, WIDTH_METERS * 0.75),
    };

    /** Total pole height from the floor, including the base. */
    public static final double POLE_HEIGHT_METERS = Units.inchesToMeters(48);

    /**
     * Square base under each pole (ESTIMATED - update when the real bases are
     * built).
     */
    public static final double POLE_BASE_SIZE_METERS = Units.inchesToMeters(14);

    public static final double POLE_BASE_HEIGHT_METERS = Units.inchesToMeters(10);

    /**
     * AprilTag IDs on each pole base, indexed like CENTER_POLES. Per base the order
     * is: +X side
     * (faces red), +Y side, -X side (faces blue), -Y side.
     */
    public static final int[][] POLE_BASE_TAG_IDS = {
            { 13, 14, 15, 16 },
            { 17, 18, 19, 20 },
            { 21, 22, 23, 24 },
    };

    /**
     * Driver-area tags near each alliance wall: {alliance wall left end, right side
     * wall, left side
     * wall}. "Left/right" for the side walls is as seen from the BLUE driver
     * station.
     */
    public static final int[] BLUE_DRIVER_AREA_TAG_IDS = { 25, 26, 27 };

    public static final int[] RED_DRIVER_AREA_TAG_IDS = { 28, 30, 29 };

    /**
     * Where the robot center should be to score on the blue Steps, facing the
     * structure.
     *
     * @param bumperToCenterMeters distance from your front bumper face to the robot
     *                             center
     */
    public static Pose2d blueStepsScoringPose(double bumperToCenterMeters) {
        return new Pose2d(
                BLUE_STEPS_FRONT_X + bumperToCenterMeters,
                WIDTH_METERS / 2,
                Rotation2d.k180deg); // robot front points at -X, toward the Steps
    }

    // ---------------------------------------------------------------------------
    // Alliance flipping. Cone Clash is ROTATIONALLY symmetric (like FRC 2026):
    // the red side is the blue side spun 180 degrees around the field center.
    // ---------------------------------------------------------------------------
    public static final class AllianceFlip {
        private AllianceFlip() {
        }

        public static Translation2d flip(Translation2d t) {
            return new Translation2d(LENGTH_METERS - t.getX(), WIDTH_METERS - t.getY());
        }

        public static Pose2d flip(Pose2d p) {
            return new Pose2d(flip(p.getTranslation()), p.getRotation().plus(Rotation2d.k180deg));
        }

        /**
         * Returns the pose for our current alliance (blue pose unchanged, red pose
         * flipped).
         */
        public static Pose2d forAlliance(Pose2d bluePose) {
            boolean isRed = DriverStation.getAlliance()
                    .orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;
            return isRed ? flip(bluePose) : bluePose;
        }

        public static Translation2d forAlliance(Translation2d blueTranslation) {
            boolean isRed = DriverStation.getAlliance()
                    .orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;
            return isRed ? flip(blueTranslation) : blueTranslation;
        }
    }
}