// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants.subsystems;

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
public final class IndexerConstants {
    /* All constants here are guesses and estimates */

    // Motor
    public static final String BUS_NAME = "rio"; // roboRIO native CAN bus; use a CANivore name for a separate bus
    public static final int DEVICE_ID = 1;
    public static final double MOI = 0.01;
    public static final double GEAR_RATIO = 5.0;
    public static final double STATOR_CURRENT_LIMIT = 60.0;
    public static final double SUPPLY_CURRENT_LIMIT = 40.0;
    public static final double SUPPLY_CURRENT_LOWER_LIMIT = 30.0;
    public static final double SUPPLY_CURRENT_LOWER_TIME = 1.0;
    public static final double INDEX_VOLTS = 6.0;
    public static final double FEED_VOLTS = 10.0;
    public static final double EJECT_VOLTS = -6.0;
    public static final double FEED_CLEAR_SECONDS = 0.2;

    // Beam break sensor
    public static final int BEAM_BREAK_CHANNEL = 1;
    public static final double BEAM_BREAK_DEBOUNCE_SECONDS = 0.15;
    public static final boolean BEAM_BREAK_FALSE_WHEN_BROKEN = true;
}
