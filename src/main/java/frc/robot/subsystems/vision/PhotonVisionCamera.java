package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.constants.subsystems.VisionConstants;

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
    public PhotonVisionCamera(String name, Transform3d offset, AprilTagFieldLayout layout) {
        super(name, offset, layout);
        camera = new PhotonCamera(name);
        estimator = new PhotonPoseEstimator(layout, offset);
    }

    /**
     * Constructor
     * 
     * @param name   name of the camera
     * @param offset robot-to-camera offset
     */
    public PhotonVisionCamera(String name, Transform3d offset) {
        this(name, offset, VisionConstants.TAG_LAYOUT);
    }

    /**
     * Updates the current pose estimate with unread camera pipeline results. This
     * method should be called every periodic.
     * 
     * @return a list of new PoseObservation instances calculated
     */
    @Override
    public List<PoseObservation> update() {
        List<PoseObservation> poseObservations = new ArrayList<>();
        List<PhotonPipelineResult> cameraResults = camera.getAllUnreadResults();
        for (PhotonPipelineResult cameraResult : cameraResults) {
            var poseEstimationResult = estimator.estimateCoprocMultiTagPose(cameraResult);
            var estimatedPose = poseEstimationResult.get();
            var targets = estimatedPose.targetsUsed;

            double totalTagDistance = 0.0;
            for (PhotonTrackedTarget target : targets) {
                totalTagDistance += target.getBestCameraToTarget().getTranslation().getNorm();
            }

            poseObservations.add(new PoseObservation(
                    estimatedPose.estimatedPose.toPose2d(),
                    estimatedPose.timestampSeconds,
                    targets.size(),
                    targets.size() > 0 ? totalTagDistance / targets.size() : 0.0,
                    targets.get(0).getPoseAmbiguity(),
                    targets.size() > 1 ? PoseType.PHOTON_MULTI_TAG : PoseType.PHOTON_SINGLE_TAG));
        }
        return poseObservations;
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
}