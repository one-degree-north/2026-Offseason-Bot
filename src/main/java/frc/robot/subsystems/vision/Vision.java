package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;
import org.photonvision.simulation.VisionSystemSim;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.subsystems.VisionConstants;
import frc.robot.subsystems.vision.VisionCamera.PoseObservation;
import frc.robot.subsystems.vision.VisionCamera.PoseType;

public class Vision extends SubsystemBase {
    @FunctionalInterface
    public interface VisionConsumer {
        void accept(Pose2d pose, double timestampSeconds, Matrix<N3, N1> stdDevs);
    }

    public enum Verdict {
        ACCEPTED,
        NO_TAGS,
        OUTSIDE_FIELD,
        TOO_FAR,
        SPINNING_TOO_FAST,
        TOO_AMBIGUOUS
    }

    private final VisionConsumer consumer;
    private final DoubleSupplier yawRateRotPerSec;
    private final List<VisionCamera> cameras;

    private final List<Alert> disconnectedAlerts = new ArrayList<>();
    private final List<StructArrayPublisher<Pose2d>> accepted = new ArrayList<>();
    private final List<StructArrayPublisher<Pose2d>> rejectedPublishers = new ArrayList<>();
    private final List<StringPublisher> verdictPublishers = new ArrayList<>();

    private VisionSystemSim visionSim;

    public Vision(VisionConsumer consumer, DoubleSupplier yawRateRotPerSec, VisionCamera... cameras) {
        this.consumer = consumer;
        this.yawRateRotPerSec = yawRateRotPerSec;
        this.cameras = List.of(cameras);
    }

    @Override
    public void periodic() {

    }

    static Verdict checkObservation(PoseObservation observation, double yawRateRotPerSec) {
        // Rule 1: a pose with no tags behind it is meaningless
        if (observation.tagCount() == 0) {
            return Verdict.NO_TAGS;
        }

        // Rule 2: the robot can't be outside the field
        Pose2d pose = observation.pose();
        double fieldLength = VisionConstants.TAG_LAYOUT.getFieldLength();
        double fieldWidth = VisionConstants.TAG_LAYOUT.getFieldWidth();
        if (pose.getX() < 0.0 || pose.getY() > fieldWidth || pose.getX() > fieldLength || pose.getY() < 0.0) {
            return Verdict.OUTSIDE_FIELD;
        }

        // Rule 3: single-tag PhotonVision pose too ambiguous
        if (observation.type() == PoseType.PHOTON_SINGLE_TAG
                && observation.ambiguity() > VisionConstants.MAX_AMBIGUITY) {
            return Verdict.TOO_AMBIGUOUS;
        }

        // Rule 4: one far-away tag is too inaccurate
        if (observation.tagCount() == 1
                && observation.averageTagDistance() > VisionConstants.MAX_SINGLE_TAG_DISTANCE_METERS) {
            return Verdict.TOO_FAR;
        }

        // Rule 5: MegaTag2 goes wrong while spinning fast (its heading input is stale)
        if (observation.type() == PoseType.MEGA_TAG_2
                && Math.abs(yawRateRotPerSec) > VisionConstants.MAX_YAW_RATE_ROT_PER_SEC) {
            return Verdict.SPINNING_TOO_FAST;
        }

        return Verdict.ACCEPTED;
    }

    static Matrix<N3, N1> computeStdDevs(PoseObservation observation, double trustFactor) {
        double distance = Math.max(observation.averageTagDistance(), VisionConstants.MIN_TAG_DISTANCE_METERS);
        double factor = (distance * distance) / observation.tagCount() * trustFactor;
        double linearStdDev = VisionConstants.LINEAR_STD_DEV_BASELINE_METERS * factor;
        double angularStdDev = VisionConstants.ANGULAR_STD_DEV_BASELINE_RADIANS * factor;

        if (observation.type() == PoseType.MEGA_TAG_2) {
            linearStdDev *= VisionConstants.MEGATAG2_LINEAR_FACTOR;
            angularStdDev = VisionConstants.IGNORE_STD_DEV;
        }

        return VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev);
    }
}
