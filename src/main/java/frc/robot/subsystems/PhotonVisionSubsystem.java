package frc.robot.subsystems;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.EstimatedRobotPose;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/**
 * PhotonVision pose estimation subsystem.
 * Feeds pose measurements whenever AprilTags are visible, with a
 * minimum tag count and ambiguity threshold for reliability.
 */
public class PhotonVisionSubsystem extends SubsystemBase {

    // Minimum tags required to trust a pose estimate (when multi-tag PnP unavailable)
    private static final int MIN_TAGS = 2;

    private final PhotonCamera m_rightCamera = new PhotonCamera("photon-right");
    private final PhotonCamera m_leftCamera  = new PhotonCamera("photon-left");

    private final PhotonPoseEstimator m_rightEstimator;
    private final PhotonPoseEstimator m_leftEstimator;

    private final SwerveSubsystem m_drivebase;

    public PhotonVisionSubsystem(SwerveSubsystem drivebase) {
        m_drivebase = drivebase;

        m_rightEstimator = new PhotonPoseEstimator(
            VisionConstants.APRIL_TAG_FIELD_LAYOUT,
            VisionConstants.RIGHT_CAMERA_TRANSFORM
        );
        m_leftEstimator = new PhotonPoseEstimator(
            VisionConstants.APRIL_TAG_FIELD_LAYOUT,
            VisionConstants.LEFT_CAMERA_TRANSFORM
        );
    }

    @Override
    public void periodic() {
        updateFromCamera(m_rightCamera, m_rightEstimator);
        updateFromCamera(m_leftCamera, m_leftEstimator);
    }

    private void updateFromCamera(PhotonCamera camera, PhotonPoseEstimator estimator) {
        var results = camera.getAllUnreadResults();
        for (var result : results) {
            if (!result.hasTargets()) continue;

            // Reliability gate: multi-tag PnP is most accurate;
            // for single-tag, only accept if the result has low estimated pose ambiguity
            var poseOptional = estimator.estimateCoprocMultiTagPose(result);
            if (poseOptional.isEmpty()) {
                // Multi-tag unavailable — require >= 2 targets for reliability
                if (result.getTargets().size() < MIN_TAGS) continue;
                poseOptional = estimator.estimateLowestAmbiguityPose(result);
            }

            if (poseOptional.isPresent()) {
                EstimatedRobotPose poseEstimate = poseOptional.get();
                m_drivebase.getSwerveDrive().addVisionMeasurement(
                    poseEstimate.estimatedPose.toPose2d(),
                    poseEstimate.timestampSeconds
                );
            }
        }
    }
}
