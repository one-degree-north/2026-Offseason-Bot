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

    public enum Vedict {
        ACCEPTED,
        NO_TAGS,
        OUTSIDE_FIELD,
        TOO_FAR,
        SPINNING_TOO_FAST
    }

    private final VisionConsumer consumer;
    private final DoubleSupplier yawRateRotPerseconds;
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
        // Telemetry and simulation setup are added in Steps 6 and 8.
    }

    @Override
    public void periodic() {

    }

    static Verdict checkObservation(PoseObservation observation, double yawRateRotPerSec) {
        return Verdict.ACCEPTED;
    }

    static Matrix<N3, N1> computeStdDevs(PoseObservation observation, double trustFactor) {
        return VecBuilder.fill(1, 1, 1);
    }

}
