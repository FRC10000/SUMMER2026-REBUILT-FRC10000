package frc.robot.subsystems;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
 * Logs raw detection data to NetworkTables for AdvantageScope visualization.
 */
public class PhotonVisionSubsystem extends SubsystemBase {

    // Minimum tags required to trust a pose estimate (when multi-tag PnP unavailable)
    private static final int MIN_TAGS = 2;
    // Single-tag pose accepted only within this distance (meters)
    private static final double SINGLE_TAG_MAX_DISTANCE = 4.0;

    private final PhotonCamera m_rightCamera = new PhotonCamera("photon-right");
    private final PhotonCamera m_leftCamera  = new PhotonCamera("photon-left");

    private final PhotonPoseEstimator m_rightEstimator;
    private final PhotonPoseEstimator m_leftEstimator;

    private final SwerveSubsystem m_drivebase;

    // AdvantageScope logging via NT4
    private final NetworkTableEntry m_leftTargetsEntry;
    private final NetworkTableEntry m_rightTargetsEntry;
    private final NetworkTableEntry m_leftPoseEntry;
    private final NetworkTableEntry m_rightPoseEntry;
    private final NetworkTableEntry m_leftTagCountEntry;
    private final NetworkTableEntry m_rightTagCountEntry;
    private final NetworkTableEntry m_leftFedEntry;
    private final NetworkTableEntry m_rightFedEntry;

    // AdvantageScope Pose3d logging (vision poses + camera transforms)
    private final NetworkTableEntry m_leftVisionPose3d;
    private final NetworkTableEntry m_rightVisionPose3d;
    private final NetworkTableEntry m_leftCameraPose3d;
    private final NetworkTableEntry m_rightCameraPose3d;
    private final NetworkTableEntry m_leftTargetIDs;
    private final NetworkTableEntry m_rightTargetIDs;

    private int m_logCounter = 0;

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

        // Create NT4 tables for AdvantageScope
        NetworkTable photonTable = NetworkTableInstance.getDefault().getTable("PhotonVision");

        m_leftTargetsEntry  = photonTable.getEntry("Left/Targets");
        m_rightTargetsEntry = photonTable.getEntry("Right/Targets");
        m_leftPoseEntry     = photonTable.getEntry("Left/EstimatedPose");
        m_rightPoseEntry    = photonTable.getEntry("Right/EstimatedPose");
        m_leftTagCountEntry = photonTable.getEntry("Left/TagCount");
        m_rightTagCountEntry= photonTable.getEntry("Right/TagCount");
        m_leftFedEntry      = photonTable.getEntry("Left/FedToEstimator");
        m_rightFedEntry     = photonTable.getEntry("Right/FedToEstimator");

        // AdvantageScope Pose3d entries (vision poses, camera transforms, target IDs)
        m_leftVisionPose3d   = photonTable.getEntry("Left/VisionPose3d");
        m_rightVisionPose3d  = photonTable.getEntry("Right/VisionPose3d");
        m_leftCameraPose3d   = photonTable.getEntry("Left/CameraPose3d");
        m_rightCameraPose3d  = photonTable.getEntry("Right/CameraPose3d");
        m_leftTargetIDs      = photonTable.getEntry("Left/TargetIDs");
        m_rightTargetIDs     = photonTable.getEntry("Right/TargetIDs");

        // Publish camera transforms as Pose3d (full rotation, for AdvantageScope 3D overlay)
        m_leftCameraPose3d.setDoubleArray(pose3dToArray(
            new Pose3d(
                VisionConstants.LEFT_CAMERA_TRANSFORM.getTranslation(),
                VisionConstants.LEFT_CAMERA_TRANSFORM.getRotation()
            )
        ));
        m_rightCameraPose3d.setDoubleArray(pose3dToArray(
            new Pose3d(
                VisionConstants.RIGHT_CAMERA_TRANSFORM.getTranslation(),
                VisionConstants.RIGHT_CAMERA_TRANSFORM.getRotation()
            )
        ));

        // Legacy camera transform entries (translation only, kept for backward compat)
        var leftT = VisionConstants.LEFT_CAMERA_TRANSFORM.getTranslation();
        var rightT = VisionConstants.RIGHT_CAMERA_TRANSFORM.getTranslation();
        photonTable.getEntry("Left/CameraTransform").setDoubleArray(
            new double[]{leftT.getX(), leftT.getY(), leftT.getZ()});
        photonTable.getEntry("Right/CameraTransform").setDoubleArray(
            new double[]{rightT.getX(), rightT.getY(), rightT.getZ()});
    }

    @Override
    public void periodic() {
        boolean leftFed = updateFromCamera(m_leftCamera, m_leftEstimator,
            m_leftTargetsEntry, m_leftPoseEntry, m_leftTagCountEntry,
            m_leftVisionPose3d, m_leftTargetIDs);
        boolean rightFed = updateFromCamera(m_rightCamera, m_rightEstimator,
            m_rightTargetsEntry, m_rightPoseEntry, m_rightTagCountEntry,
            m_rightVisionPose3d, m_rightTargetIDs);
        m_leftFedEntry.setBoolean(leftFed);
        m_rightFedEntry.setBoolean(rightFed);

        // Log robot pose + vision pose for AdvantageScope field overlay
        if (++m_logCounter >= 5) { // every 100ms
            m_logCounter = 0;
            Pose2d robotPose = m_drivebase.getPose();
            SmartDashboard.putNumber("PhotonVision/RobotPoseX", robotPose.getX());
            SmartDashboard.putNumber("PhotonVision/RobotPoseY", robotPose.getY());
            SmartDashboard.putNumber("PhotonVision/RobotHeading", robotPose.getRotation().getDegrees());
        }
    }

    /**
     * Process camera results, log raw targets, and feed pose to estimator.
     * @return true if a pose measurement was fed to the drivetrain estimator
     */
    private boolean updateFromCamera(PhotonCamera camera, PhotonPoseEstimator estimator,
                                      NetworkTableEntry targetsEntry, NetworkTableEntry poseEntry,
                                      NetworkTableEntry tagCountEntry,
                                      NetworkTableEntry visionPose3dEntry,
                                      NetworkTableEntry targetIDsEntry) {
        var results = camera.getAllUnreadResults();
        boolean fedAny = false;

        for (var result : results) {
            if (!result.hasTargets()) {
                targetsEntry.setDoubleArray(new double[0]);
                tagCountEntry.setInteger(0);
                poseEntry.setDoubleArray(new double[0]);
                visionPose3dEntry.setDoubleArray(new double[0]);
                targetIDsEntry.setIntegerArray(new long[0]);
                continue;
            }

            // Log all detected targets: [tagID, yaw, pitch, area, dist] per target
            var targets = result.getTargets();
            tagCountEntry.setInteger(targets.size());

            // Log detected tag IDs for AdvantageScope AprilTag overlay
            long[] tagIDs = new long[targets.size()];
            double[] targetData = new double[targets.size() * 5];
            for (int i = 0; i < targets.size(); i++) {
                var t = targets.get(i);
                tagIDs[i] = t.getFiducialId();
                int base = i * 5;
                targetData[base]     = t.getFiducialId();
                targetData[base + 1] = t.getYaw();         // horizontal angle deg
                targetData[base + 2] = t.getPitch();       // vertical angle deg
                targetData[base + 3] = t.getArea();         // target area (0-1)
                targetData[base + 4] = t.getBestCameraToTarget().getTranslation().getNorm(); // distance m
            }
            targetsEntry.setDoubleArray(targetData);
            targetIDsEntry.setIntegerArray(tagIDs);

            // Reliability gate: multi-tag PnP is most accurate;
            // for single-tag, only accept if within range
            var poseOptional = estimator.estimateCoprocMultiTagPose(result);
            if (poseOptional.isEmpty()) {
                if (result.getTargets().size() < MIN_TAGS) {
                    // Single tag — accept only if close enough
                    var tag = result.getTargets().get(0);
                    double dist = tag.getBestCameraToTarget().getTranslation().getNorm();
                    if (dist > SINGLE_TAG_MAX_DISTANCE) {
                        poseEntry.setDoubleArray(new double[0]);
                        visionPose3dEntry.setDoubleArray(new double[0]);
                        continue;
                    }
                }
                poseOptional = estimator.estimateLowestAmbiguityPose(result);
            }

            if (poseOptional.isPresent()) {
                EstimatedRobotPose poseEstimate = poseOptional.get();
                m_drivebase.getSwerveDrive().addVisionMeasurement(
                    poseEstimate.estimatedPose.toPose2d(),
                    poseEstimate.timestampSeconds
                );
                // Log estimated pose: [x, y, yawRadians] (legacy format)
                Pose3d est = poseEstimate.estimatedPose;
                poseEntry.setDoubleArray(new double[]{
                    est.getX(), est.getY(),
                    est.getRotation().getZ()
                });
                // Log Pose3d for AdvantageScope (x, y, z, qw, qx, qy, qz)
                visionPose3dEntry.setDoubleArray(pose3dToArray(est));
                fedAny = true;
            } else {
                poseEntry.setDoubleArray(new double[0]);
                visionPose3dEntry.setDoubleArray(new double[0]);
            }
        }
        return fedAny;
    }

    /** Serialize a Pose3d to a flat double array [x, y, z, qw, qx, qy, qz]. */
    private static double[] pose3dToArray(Pose3d pose) {
        var q = pose.getRotation().getQuaternion();
        return new double[]{
            pose.getX(), pose.getY(), pose.getZ(),
            q.getW(), q.getX(), q.getY(), q.getZ()
        };
    }
}
