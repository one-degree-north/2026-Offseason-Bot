
package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.subsystems.VisionConstants;
import frc.robot.subsystems.vision.Vision.Verdict;
import frc.robot.subsystems.vision.VisionCamera.PoseObservation;
import frc.robot.subsystems.vision.VisionCamera.PoseType;

class VisionTest {
    private static final double EPSILON = 1e-9; // "close enough" for comparing doubles

    @BeforeEach
    void setUp() {
        // Starts WPILib's simulated hardware layer. Needed because Vision is a
        // subsystem and creates Alerts and NetworkTables publishers.
        assertTrue(HAL.initialize(500, 0));
    }

    @AfterEach
    void tearDown() {
        CommandScheduler.getInstance().unregisterAllSubsystems();
    }

    /** Builds an observation at (x, y) facing 0°, with a timestamp of 1.0 s. */
    private static PoseObservation observation(
            double x, double y, int tags, double distance, double ambiguity, PoseType type) {
        return new PoseObservation(new Pose2d(x, y, new Rotation2d()), 1.0, tags, distance,
                ambiguity, type);
    }

    // ===================== checkObservation =====================
    @Test
    void acceptsGoodMultiTag() {
        var obs = observation(5.0, 4.0, 2, 2.0, 0.0, PoseType.PHOTON_MULTI_TAG);
        assertEquals(Verdict.ACCEPTED, Vision.checkObservation(obs, 0.0));
    }

    @Test
    void rejectFarTags() {
        var obs = observation(5.0, 4.0, 1, 4.1, 0.0, PoseType.PHOTON_SINGLE_TAG);
        assertEquals(Verdict.TOO_FAR, Vision.checkObservation(obs, 0.0));
    }

    @Test
    void rejectsNoTags() {
        var obs = observation(5.0, 4.0, 0, 2.0, 0.0, PoseType.MEGA_TAG_2);
        assertEquals(Verdict.NO_TAGS, Vision.checkObservation(obs, 0.0));
    }

    @Test
    void rejectsPoseOutsideField() {
        var obs = observation(-1.0, 4.0, 2, 2.0, 0.0, PoseType.PHOTON_MULTI_TAG);
        assertEquals(Verdict.OUTSIDE_FIELD, Vision.checkObservation(obs, 0.0));
    }

    @Test
    void rejectsAmbiguousSingleTag() {
        var obs = observation(5.0, 4.0, 1, 2.0, 0.5, PoseType.PHOTON_SINGLE_TAG);
        assertEquals(Verdict.TOO_AMBIGUOUS, Vision.checkObservation(obs, 0.0));
    }

    @Test
    void ignoresAmbiguityForMultiTag() {
        var obs = observation(5.0, 4.0, 2, 2.0, 0.9, PoseType.PHOTON_MULTI_TAG);
        assertEquals(Verdict.ACCEPTED, Vision.checkObservation(obs, 0.0));
    }

    @Test
    void rejectsFarSingleTag() {
        var obs = observation(5.0, 4.0, 1, 5.0, 0.05, PoseType.PHOTON_SINGLE_TAG);
        assertEquals(Verdict.TOO_FAR, Vision.checkObservation(obs, 0.0));
    }

    @Test
    void rejectsMegaTag2WhileSpinningFast() {
        var obs = observation(5.0, 4.0, 2, 2.0, 0.0, PoseType.MEGA_TAG_2);
        assertEquals(Verdict.SPINNING_TOO_FAST, Vision.checkObservation(obs, 3.0));
    }

    @Test
    void spinningDoesNotAffectPhotonVision() {

        var obs = observation(5.0, 4.0, 2, 2.0, 0.0, PoseType.PHOTON_MULTI_TAG);
        assertEquals(Verdict.ACCEPTED, Vision.checkObservation(obs, 3.0));
    }

    // ===================== computeStdDevs =====================
    @Test
    void oneTagAtOneMeterGivesBaseline() {
        var obs = observation(5.0, 4.0, 1, 1.0, 0.0, PoseType.PHOTON_SINGLE_TAG);
        Matrix<N3, N1> std = Vision.computeStdDevs(obs, 1.0);
        assertEquals(VisionConstants.LINEAR_STD_DEV_BASELINE_METERS, std.get(0, 0), EPSILON);
        assertEquals(VisionConstants.LINEAR_STD_DEV_BASELINE_METERS, std.get(1, 0), EPSILON);
        assertEquals(VisionConstants.ANGULAR_STD_DEV_BASELINE_RADIANS, std.get(2, 0), EPSILON);
    }

    @Test
    void doublingDistanceQuadruplesStdDev() {
        var near = Vision.computeStdDevs(observation(5, 4, 1, 1.0, 0, PoseType.PHOTON_SINGLE_TAG),
                1.0);
        var far = Vision.computeStdDevs(observation(5, 4, 1, 2.0, 0, PoseType.PHOTON_SINGLE_TAG),
                1.0);
        assertEquals(4.0 * near.get(0, 0), far.get(0, 0), EPSILON);
    }

    @Test
    void twoTagsHalveStdDev() {
        var one = Vision.computeStdDevs(observation(5, 4, 1, 3.0, 0, PoseType.PHOTON_SINGLE_TAG),
                1.0);
        var two = Vision.computeStdDevs(observation(5, 4, 2, 3.0, 0, PoseType.PHOTON_MULTI_TAG),
                1.0);
        assertEquals(one.get(0, 0) / 2.0, two.get(0, 0), EPSILON);
    }

    @Test
    void twoTagsStdDev() {
        var std = Vision.computeStdDevs(observation(5, 4, 2, 2.0, 0, PoseType.MEGA_TAG_2),
                1.0);
        assertEquals(std.get(0, 0), 0.02, EPSILON);
        assertEquals(std.get(1, 0), 0.02, EPSILON);
    }

    @Test
    void megaTag2IgnoresHeading() {
        var std = Vision.computeStdDevs(observation(5, 4, 1, 1.0, 0, PoseType.MEGA_TAG_2), 1.0);
        assertEquals(VisionConstants.LINEAR_STD_DEV_BASELINE_METERS
                * VisionConstants.MEGATAG2_LINEAR_FACTOR, std.get(0, 0), EPSILON);
        assertEquals(VisionConstants.IGNORE_STD_DEV, std.get(2, 0), EPSILON);
    }

    @Test
    void tinyDistanceIsClampedSoStdDevIsNeverZero() {
        var std = Vision.computeStdDevs(observation(5, 4, 1, 0.0, 0, PoseType.PHOTON_SINGLE_TAG),
                1.0);
        assertTrue(std.get(0, 0) > 0.0);
    }

    @Test
    void trustFactorScalesStdDev() {
        var normal = Vision.computeStdDevs(observation(5, 4, 1, 2.0, 0, PoseType.PHOTON_SINGLE_TAG),
                1.0);
        var worse = Vision.computeStdDevs(observation(5, 4, 1, 2.0, 0, PoseType.PHOTON_SINGLE_TAG),
                1.5);
        assertEquals(1.5 * normal.get(0, 0), worse.get(0, 0), EPSILON);
    }

    // ===================== periodic() with fake cameras =====================
    /** A pretend camera: returns whatever observations we queue up, once. */
    private static class FakeCamera extends VisionCamera {
        List<PoseObservation> queued = new ArrayList<>();

        FakeCamera(String name) {
            super(name, new Transform3d(), VisionConstants.TAG_LAYOUT, 1.0);

        }

        @Override
        public List<PoseObservation> update() {
            List<PoseObservation> out = queued;
            queued = new ArrayList<>(); // like a real camera: each frame is returned once
            return out;
        }

        @Override
        public boolean isConnected() {
            return true;
        }
    }

    @Test
    void sendsOnlyAcceptedObservationsToDrivetrain() {
        List<Pose2d> received = new ArrayList<>();
        FakeCamera camera = new FakeCamera("fake");
        Vision vision = new Vision((pose, time, std) -> received.add(pose), () -> 0.0, camera);
        camera.queued.add(observation(5.0, 4.0, 2, 2.0, 0.0, PoseType.PHOTON_MULTI_TAG)); // good
        camera.queued.add(observation(-3.0, 4.0, 2, 2.0, 0.0, PoseType.PHOTON_MULTI_TAG)); // outside field
        vision.periodic();
        assertEquals(1, received.size());
        assertEquals(5.0, received.get(0).getX(), EPSILON);
    }

    @Test
    void eachObservationIsSentOnlyOnce() {
        List<Pose2d> received = new ArrayList<>();
        FakeCamera camera = new FakeCamera("fake");
        Vision vision = new Vision((pose, time, std) -> received.add(pose), () -> 0.0, camera);
        camera.queued.add(observation(5.0, 4.0, 2, 2.0, 0.0, PoseType.PHOTON_MULTI_TAG));
        vision.periodic();
        vision.periodic(); // nothing new this loop
        assertEquals(1, received.size());
    }

    @Test
    void handlesMultipleCameras() {
        List<Pose2d> received = new ArrayList<>();
        FakeCamera front = new FakeCamera("front");
        FakeCamera back = new FakeCamera("back");
        Vision vision = new Vision((pose, time, std) -> received.add(pose), () -> 0.0, front, back);
        front.queued.add(observation(5.0, 4.0, 2, 2.0, 0.0, PoseType.PHOTON_MULTI_TAG));
        back.queued.add(observation(6.0, 3.0, 1, 1.5, 0.0, PoseType.MEGA_TAG_2));
        vision.periodic();
        assertEquals(2, received.size());
    }

    @Test
    void passesTheCameraTimestampThrough() {
        List<Double> times = new ArrayList<>();
        FakeCamera camera = new FakeCamera("fake");
        Vision vision = new Vision((pose, time, std) -> times.add(time), () -> 0.0, camera);
        camera.queued.add(observation(5.0, 4.0, 2, 2.0, 0.0, PoseType.PHOTON_MULTI_TAG));
        vision.periodic();

        assertEquals(1.0, times.get(0), EPSILON); // the photo's time, not "now"
    }
}