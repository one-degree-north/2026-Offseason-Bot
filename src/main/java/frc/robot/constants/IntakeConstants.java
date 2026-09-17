package frc.robot.constants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class IntakeConstants {
    /* All constants here are guesses and estimates */

    // Motor
    public static final int BUS_ID = 0;
    public static final int DEVICE_ID = 0;
    public static final double MOI = 0.01;
    public static final double GEAR_RATIO = 5.0;
    public static final double STATOR_CURRENT_LIMIT = 60.0;
    public static final double SUPPLY_CURRENT_LIMIT = 40.0;
    public static final double SUPPLY_CURRENT_LOWER_LIMIT = 30.0;
    public static final double SUPPLY_CURRENT_LOWER_TIME = 1.0;
    public static final double INTAKE_VOLTS = 8.0;
    public static final double EJECT_VOLTS = -8.0;
    public static final double HOLD_VOLTS = 1.0;
}
