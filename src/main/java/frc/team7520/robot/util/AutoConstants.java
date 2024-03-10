package frc.team7520.robot.util;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

/**
 * Container class for auto constants.
 */
public class AutoConstants {

    // Pathplanner output folder should be src/main/deploy/pathplanner
    public static final String TRAJECTORY_DIRECTORY = "pathplanner/paths/";

    public static final double MAX_AUTO_SPEED = 4.3; // m/s
    public static final double MAX_AUTO_ACCEL = 2.1; // m/s^2

    public static final PIDConstants translationConstants = new PIDConstants(0,0,0);
    public static final PIDConstants rotationConstants = new PIDConstants(0,0,0);

    public static final ReplanningConfig replanningConfig = new ReplanningConfig(true, false);

    public static final HolonomicPathFollowerConfig config = new HolonomicPathFollowerConfig(
        translationConstants,
        rotationConstants,
        MAX_AUTO_SPEED,
        0.7400, // Math.sqrt(2)* wheelBase / 2,
        replanningConfig
    );

    private AutoConstants() {
    }
}