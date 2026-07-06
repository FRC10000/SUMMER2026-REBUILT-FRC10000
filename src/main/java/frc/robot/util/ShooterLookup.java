package frc.robot.util;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.Constants.ShooterLookupConstants;

/**
 * Interpolation utility that converts distance (meters) to pivot angle and flywheel RPM.
 * Values are loaded from Constants.ShooterLookupArrays and can be tuned on the real robot.
 */
public class ShooterLookup {

    private static final InterpolatingDoubleTreeMap distanceToPivot = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap distanceToRPM = new InterpolatingDoubleTreeMap();

    static {
        for (int i = 0; i < ShooterLookupConstants.DISTANCE_TO_PIVOT_DISTANCE.length; i++) {
            distanceToPivot.put(
                ShooterLookupConstants.DISTANCE_TO_PIVOT_DISTANCE[i],
                ShooterLookupConstants.DISTANCE_TO_PIVOT_ANGLE[i]
            );
        }
        for (int i = 0; i < ShooterLookupConstants.DISTANCE_TO_RPM_DISTANCE.length; i++) {
            distanceToRPM.put(
                ShooterLookupConstants.DISTANCE_TO_RPM_DISTANCE[i],
                ShooterLookupConstants.DISTANCE_TO_RPM_RPM[i]
            );
        }
    }

    /** Returns interpolated pivot angle (degrees) for a given distance (meters). */
    public static double getPivotAngle(double distanceMeters) {
        return distanceToPivot.get(distanceMeters);
    }

    /** Returns interpolated flywheel RPM for a given distance (meters). */
    public static double getRPM(double distanceMeters) {
        return distanceToRPM.get(distanceMeters);
    }
}
