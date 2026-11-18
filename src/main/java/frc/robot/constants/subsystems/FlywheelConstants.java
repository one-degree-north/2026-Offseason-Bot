package frc.robot.constants.subsystems;

/**
 * Constants for the flywheel shooter. All values here are guesses and
 * estimates.
 */
public final class FlywheelConstants {
    private FlywheelConstants() {
    }

    // Motor
    public static final int DEVICE_ID = 20;
    public static final double MOI = 0.004; // kg*m^2
    public static final double GEAR_RATIO = 1.0; // 1.0 = direct drive
    public static final double STATOR_CURRENT_LIMIT = 80.0;
    public static final double SUPPLY_CURRENT_LIMIT = 40.0;
    public static final double SUPPLY_CURRENT_LOWER_LIMIT = 30.0;
    public static final double SUPPLY_CURRENT_LOWER_TIME = 1.0;

    // Motor specs (used to calculate kV)
    public static final double NOMINAL_VOLTAGE = 12.0;
    public static final double KRAKEN_X60_FREE_SPEED_RPS = 100.0; // ~6000 RPM

    // Closed-loop gains (Slot 0). kV is calculated in code, not stored here.
    public static final double KS = 0.0; // volts, ~0 in sim because the sim has no friction
    public static final double KP = 0.0; // TODO: (Level 2): tune this in simulation
    public static final double KA = 0.0;

    public static final double TOLERANCE_RPS = 2.0;

    // Test values
    public static final double TEST_VOLTS = 6.0;
    public static final double TEST_TARGET_RPS = 60.0;
}
