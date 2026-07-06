package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import java.util.Optional;
import swervelib.SwerveDrive;

/**
 * Limelight 3G dual-camera vision subsystem.
 * Back Limelight: odometry via Megatag 2 pose estimation.
 * Front Limelight (on turret): targeting via tx/ty offsets.
 */
public class LimelightVisionSubsystem extends SubsystemBase {

    private final SwerveDrive swerveDrive;

    // NetworkTables references
    private final NetworkTable backTable;
    private final NetworkTable frontTable;

    // Megatag 2 botpose entries (DoubleArray: [x, y, z, roll, pitch, yaw, latency, tagCount, ...])
    private final DoubleArrayEntry backBotposeEntry;

    // Targeting entries from front Limelight
    private final DoubleEntry frontTxEntry;
    private final DoubleEntry frontTyEntry;
    private final DoubleEntry frontTvEntry;

    // Cached targeting values
    private boolean hasTarget = false;
    private double targetTx = 0.0;
    private double targetTy = 0.0;

    // Cached pose from back Limelight
    private Optional<Pose2d> estimatedPose = Optional.empty();

    public LimelightVisionSubsystem(SwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;

        NetworkTableInstance nt = NetworkTableInstance.getDefault();

        // Back Limelight (odometry)
        backTable = nt.getTable(VisionConstants.BACK_LIMELIGHT);
        backBotposeEntry = backTable.getDoubleArrayTopic("botpose_orb").getEntry(new double[0]);

        // Front Limelight (turret targeting)
        frontTable = nt.getTable(VisionConstants.FRONT_LIMELIGHT);
        frontTxEntry = frontTable.getDoubleTopic("tx").getEntry(0.0);
        frontTyEntry = frontTable.getDoubleTopic("ty").getEntry(0.0);
        frontTvEntry = frontTable.getDoubleTopic("tv").getEntry(0.0);
    }

    @Override
    public void periodic() {
        updateOdometry();
        updateTargeting();
        publishSmartDashboard();
    }

    /** Read Megatag 2 pose from back Limelight and inject into swerve pose estimator. */
    private void updateOdometry() {
        double[] botpose = backBotposeEntry.get();

        // botpose_orb format: [x, y, z, roll, pitch, yaw, latency, tagCount, ...]
        if (botpose.length < 7) {
            estimatedPose = Optional.empty();
            return;
        }

        int tagCount = botpose.length > 7 ? (int) botpose[7] : 0;
        double latency = botpose[6]; // milliseconds

        if (tagCount <= 0) {
            estimatedPose = Optional.empty();
            return;
        }

        double x = botpose[0];
        double y = botpose[1];
        double yaw = botpose[5];

        Pose2d pose = new Pose2d(x, y, Rotation2d.fromDegrees(yaw));
        estimatedPose = Optional.of(pose);

        // Calculate timestamp: current time minus pipeline latency
        double timestamp = edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - (latency / 1000.0);

        // Choose standard deviations based on tag count
        double[] stdDevs = tagCount > 1
            ? VisionConstants.MULTI_TAG_STD_DEV
            : VisionConstants.SINGLE_TAG_STD_DEV;

        swerveDrive.addVisionMeasurement(pose, timestamp,
            VecBuilder.fill(stdDevs[0], stdDevs[1], stdDevs[2]));
    }

    /** Read tx, ty, tv from front Limelight (turret-mounted). */
    private void updateTargeting() {
        hasTarget = frontTvEntry.get() > 0.5;
        targetTx = frontTxEntry.get();
        targetTy = frontTyEntry.get();
    }

    private void publishSmartDashboard() {
        SmartDashboard.putBoolean("Vision/Has Target", hasTarget);
        SmartDashboard.putNumber("Vision/Turret Tx", targetTx);
        SmartDashboard.putNumber("Vision/Turret Ty", targetTy);
        SmartDashboard.putNumber("Vision/Estimated Distance", getEstimatedDistance());
        if (estimatedPose.isPresent()) {
            Pose2d p = estimatedPose.get();
            SmartDashboard.putNumber("Vision/Pose X", p.getX());
            SmartDashboard.putNumber("Vision/Pose Y", p.getY());
            SmartDashboard.putNumber("Vision/Pose Rot", p.getRotation().getDegrees());
        }
    }

    // --- Getters ---

    public boolean hasTarget() {
        return hasTarget;
    }

    /** Horizontal offset from crosshair to target in degrees. Positive = target is to the right. */
    public double getTargetTx() {
        return targetTx;
    }

    /** Vertical offset from crosshair to target in degrees. Positive = target is above center. */
    public double getTargetTy() {
        return targetTy;
    }

    /**
     * Estimates horizontal distance from the turret camera to the target.
     * Uses trig: distance = (camera_height - target_height) / tan(mount_angle + ty)
     */
    public double getEstimatedDistance() {
        double heightDiff = VisionConstants.FRONT_CAM_HEIGHT_METERS - VisionConstants.TARGET_HEIGHT_METERS;
        double totalAngle = VisionConstants.FRONT_CAM_MOUNT_ANGLE_DEG + targetTy;
        double tanAngle = Math.tan(Math.toRadians(totalAngle));
        if (Math.abs(tanAngle) < 0.001) {
            return 0.0;
        }
        return heightDiff / tanAngle;
    }

    public Optional<Pose2d> getEstimatedPose() {
        return estimatedPose;
    }
}
