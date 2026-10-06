package first.robot.utils;

import org.wpilib.math.interpolation.InterpolatingDoubleTreeMap;
import org.wpilib.math.util.Units;

/* Holds a table of distances and shooter RPMs,
   allowing for quite, measured motor speeds
   for dynamic shots */
public class ShooterTree {

    // -- Shooter Tables and Maps -- //

    // The following table was calculated via kinematics; all values need
    // to be tuned and calibrated properly
    private static final double[][] DISTANCE_RPM_TABLE = new double[][] {
        // Ft, RPM
        // This is intended to use Motor RPMs; should it be desired
        // flywheel RPM instead??
        {3.0, 4170.0},
        //{3.0, 300.0},
        {5.0, 5630.0},
        {7.0, 6035.0},
        {9.0, 6395.0},
        {11.0, 6725.0},
        {13.0, 7025.0},
        {15.0, 7305.0},
        {17.0, 7570.0}
    };

    // Create the actual maps
    private static final InterpolatingDoubleTreeMap rpmMap = new InterpolatingDoubleTreeMap();
    static {
        for (double[] entry : DISTANCE_RPM_TABLE) {
            rpmMap.put(Units.feetToMeters(entry[0]), entry[1]);
        }
    }
    
    // Constants for referencing tables
    public static final int RPM_MAP = 0;

    private static final InterpolatingDoubleTreeMap[] shooterTables = new InterpolatingDoubleTreeMap[] {
        rpmMap
    };


    // Getters
    public static double getValue(double distance, int table) {
        return shooterTables[table].get(distance);
    }

    // Eh, why not make it easier on us?
    public static double getShooterRPM(double distance) {
        return getValue(distance, RPM_MAP);
    }
}
