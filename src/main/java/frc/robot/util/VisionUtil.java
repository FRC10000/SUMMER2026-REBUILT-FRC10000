package frc.robot.util;

import frc.robot.Constants.ShooterLookupConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.util.LimelightHelpers.RawFiducial;

/**
 * Shared vision utility methods used by AutoAim, AutoShoot, and AutoAimAndShoot commands.
 */
public final class VisionUtil {

    private VisionUtil() {}

    /**
     * Find the best target from limelight fiducials.
     * First tries the Limelight's primary tracked target, then falls back to nearest by distance.
     */
    public static RawFiducial findNearestTarget(RawFiducial[] fiducials, String limelightName, int[] targetTagIds) {
        if (fiducials == null || fiducials.length == 0) {
            return null;
        }

        int primaryId = (int) LimelightHelpers.getFiducialID(limelightName);

        // 先尝试匹配 Limelight 的 primary 目标
        for (RawFiducial f : fiducials) {
            if (f.id == primaryId && isTargetTag(f.id, targetTagIds)) {
                return f;
            }
        }

        // 按距离兜底
        RawFiducial nearest = null;
        double minDist = Double.MAX_VALUE;
        for (RawFiducial f : fiducials) {
            if (!isTargetTag(f.id, targetTagIds)) continue;
            if (f.distToRobot < minDist) {
                minDist = f.distToRobot;
                nearest = f;
            }
        }
        return nearest;
    }

    public static boolean isTargetTag(int id, int[] targetTagIds) {
        for (int tid : targetTagIds) {
            if (id == tid) return true;
        }
        return false;
    }

    /**
     * Compute horizontal distance from limelight tync using camera mount geometry.
     */
    public static double computeHorizontalDistance(double tync) {
        double angleFromHoriz = VisionConstants.CAM_MOUNT_ANGLE_DEG + tync;
        double angleRad = Math.toRadians(angleFromHoriz);
        if (angleRad < 0.01) {
            return 5.0;
        }
        return VisionConstants.HEIGHT_DIFF_METERS / Math.tan(angleRad);
    }

    /**
     * Linear interpolation: distance → pivot angle from lookup table.
     */
    public static double lookupPivotAngle(double distance) {
        double[] distances = ShooterLookupConstants.DISTANCE_TO_PIVOT_DISTANCE;
        double[] angles = ShooterLookupConstants.DISTANCE_TO_PIVOT_ANGLE;

        if (distance <= distances[0]) return angles[0];
        if (distance >= distances[distances.length - 1]) return angles[angles.length - 1];

        for (int i = 0; i < distances.length - 1; i++) {
            if (distance >= distances[i] && distance <= distances[i + 1]) {
                double ratio = (distance - distances[i]) / (distances[i + 1] - distances[i]);
                return angles[i] + ratio * (angles[i + 1] - angles[i]);
            }
        }
        return angles[0];
    }

    /**
     * Linear interpolation: distance → flywheel RPM from lookup table.
     */
    public static double lookupFlywheelRPM(double distance) {
        double[] distances = ShooterLookupConstants.DISTANCE_TO_PIVOT_DISTANCE;
        double[] rpms = ShooterLookupConstants.DISTANCE_TO_RPM_RPM;

        if (distance <= distances[0]) return rpms[0];
        if (distance >= distances[distances.length - 1]) return rpms[rpms.length - 1];

        for (int i = 0; i < distances.length - 1; i++) {
            if (distance >= distances[i] && distance <= distances[i + 1]) {
                double ratio = (distance - distances[i]) / (distances[i + 1] - distances[i]);
                return rpms[i] + ratio * (rpms[i + 1] - rpms[i]);
            }
        }
        return rpms[0];
    }
}
