package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.constants.subsystems.VisionConstants;
import org.photonvision.EstimatedRobotPose;
import java.util.Optional;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * VisionCamera utilizing PhotonVision hardware and pose estimation
 */
public class PhotonVisionCamera extends VisionCamera {

    // PhotonCamera camera instance
    private final PhotonCamera camera;

    // PhotonPoseEstimator used to estimate field-relative pose
    private final PhotonPoseEstimator estimator;

    /**
     * Constructor
     * 
     * @param name   name of the camera
     * @param offset robot-to-camera offset
     * @param layout AprilTag field layout
     */
    public PhotonVisionCamera(String name, Transform3d offset, AprilTagFieldLayout layout, double trustFactor) {
        super(name, offset, layout, trustFactor);
        camera = new PhotonCamera(name);
        estimator = new PhotonPoseEstimator(layout, offset);
    }

    /**
     * Constructor
     * 
     * @param name   name of the camera
     * @param offset robot-to-camera offset
     */
    public PhotonVisionCamera(String name, Transform3d offset, double trustFactor) {
        this(name, offset, VisionConstants.TAG_LAYOUT, trustFactor);
    }

    /**
     * Updates the current pose estimate with unread camera pipeline results. This
     * method should be called every periodic.
     * 
     * @return a list of new PoseObservation instances calculated
     */
    @Override
    public List<PoseObservation> update() {
        List<PoseObservation> observations = new ArrayList<>();
        for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
            // 1. Try the best method first: all visible tags solved together.
            PoseType type = PoseType.PHOTON_MULTI_TAG;
            Optional<EstimatedRobotPose> estimate = estimator.estimateCoprocMultiTagPose(result);

            // 2. Not enough tags for multi-tag? Fall back to the single clearest tag.
            if (estimate.isEmpty()) {
                type = PoseType.PHOTON_SINGLE_TAG;
                estimate = estimator.estimateLowestAmbiguityPose(result);
            }
            // 3. Still nothing means no usable tags in this frame. Skip it.
            if (estimate.isEmpty() || estimate.get().targetsUsed.isEmpty()) {
                continue;
            }

            EstimatedRobotPose est = estimate.get();
            List<PhotonTrackedTarget> targets = est.targetsUsed;
            double totalDistance = 0.0;

            for (PhotonTrackedTarget target : targets) {
                totalDistance += target.getBestCameraToTarget().getTranslation().getNorm();
            }

            observations.add(new PoseObservation(
                    est.estimatedPose.toPose2d(),
                    est.timestampSeconds,
                    targets.size(),
                    totalDistance / targets.size(),
                    targets.get(0).getPoseAmbiguity(),
                    type));

        }

        return observations;

    }

    /**
     * Checks whether camera is connected to the robot.
     * 
     * @return true if camera is connected, false if disconnected
     */
    @Override
    public boolean isConnected() {
        return camera.isConnected();
    }
    
    @Override
    public void addToSimulation(VisionSystemSim sim){
        SimCameraProperties properties = new SimCameraProperties();
        properties.setCalibration(960, 720, Rotation2d.fromDegrees(90)); // resolution and field of view
        properties.setCalibError(0.35, 0.10); // realistic pixel noise
        properties.setFPS(15);
        properties.setAvgLatencyMs(50);
        properties.setLatencyStdDevMs(15);
        PhotonCameraSim cameraSim = new PhotonCameraSim(camera, properties);
        cameraSim.enableDrawWireframe(true);
        sim.addCamera(cameraSim, getOffset());
    }
}