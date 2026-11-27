package frc.robot.constants.subsystems;

import com.ctre.phoenix6.signals.NeutralModeValue;


public final class ElevatorConstants {

    private ElevatorConstants() {} 
    

    public static final int kLeaderMotorID = 5;
    public static final int kFollowerMotorID = 6;
    public static final String kCANBus = "rio";

    //TODO: verify in CAD. Set true if the two motors are mounted on opposite
    
    public static final boolean kFollowerOpposed = true;

    public static final NeutralModeValue kNeutralMode = NeutralModeValue.Brake;

    public static final double kSupplyCurrentLimitAmps = 40.0;
    public static final double kStatorCurrentLimitAmps = 80.0;

    

    public static final int kCascadeFactor = 3;
    public static final double kGearRatio = 10.0;
    public static final double kPulleyCircumferenceInches = 4.0;
    public static final double drumRadius = kPulleyCircumferenceInches / Math.PI / 2;

    public static final double kInchesPerMotorRotation =
        kCascadeFactor
            * (1.0 / kGearRatio)
            * kPulleyCircumferenceInches; 
    public static final double kCarriageMass = 15.0;

    
    // TODO: tape-measure. Used later for soft limits.
    

    public static final double kMinHeightInches = 0.0;
    public static final double kMaxHeightInches = 50.0; // TODO: tape-measure

    //Motion Magic Constant

    public static final double kP = 0.0;
    public static final double kI = 0.0;   // not provided; leave at 0
    public static final double kD = 0.0;

    public static final double kS = 0.0;
    public static final double kG = 0.21044;  
    public static final double kV = 1.193;   
    public static final double kA = 0.0;    

    
    // Motion Magic constraints 
    

    /** Cruise velocity in motor rotations per second. */
    public static final double kCruiseVelocityRotPerSec = 6.0;

    /** Cruise acceleration in motor rotations per second squared. */
    public static final double kCruiseAccelerationRotPerSecSq = 20.0;

    
    public static final double kMaxVelocityInchesPerSec =
        kCruiseVelocityRotPerSec * kInchesPerMotorRotation;        

    public static final double kMaxAccelerationInchesPerSecSq =
        kCruiseAccelerationRotPerSecSq * kInchesPerMotorRotation;  
        
    // Tolerances and setpoints
    
    public static final double kPositionToleranceInches = 0.25;
    public static final double kVelocityToleranceInchesPerSec = 0.5;

    // TODO: replace with measured scoring heights from homed zero.
    public static final double kStowedHeightInches = 0.0;
    public static final double kL1HeightInches = 10.0;
    public static final double kL2HeightInches = 25.0;
    public static final double kL3HeightInches = 40.0;


    //  Manual jog and homing

    /** Voltage used for manual jog in step 2. */
    public static final double kManualJogVolts = 3.0;

    public static final double kHomingVoltage = -1.0;
    public static final double kHomingCurrentThresholdAmps = 20.0;

    public static final double kHomingVelocityThreshold = 0.5;
    public static final double kHomingDebounceSeconds = 0.25;
}