package frc.robot.subsystems;

import frc.robot.constants.subsystems.ElevatorConstants;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;

public class ElevatorSubsystem extends SubsystemBase {
    public enum ElevatorMode {
        NORMAL,
        MANUAL
    }
    private ElevatorMode elevatorMode = ElevatorMode.NORMAL;
    private final TalonFX m_leader = new TalonFX(ElevatorConstants.kLeaderMotorID, ElevatorConstants.kCANBus);
    private final TalonFX m_follower = new TalonFX(ElevatorConstants.kFollowerMotorID, ElevatorConstants.kCANBus);

    private final TalonFXSimState m_leaderSim = m_leader.getSimState();
    private final TalonFXSimState m_followerSim = m_follower.getSimState();
    private final DCMotor m_gearbox = DCMotor.getKrakenX60(2);
    private final ElevatorSim m_elevatorSim = new ElevatorSim(m_gearbox, ElevatorConstants.kGearRatio,
            ElevatorConstants.kCarriageMass, Units.inchesToMeters(ElevatorConstants.drumRadius),
            Units.inchesToMeters(ElevatorConstants.kMinHeightInches),
            Units.inchesToMeters(ElevatorConstants.kMaxHeightInches), true, 0);

    private double elevatorHeight = 0.0;
    private double elevatorVelocity = 0.0;
    private double elevatorAcceleration = 0.0;
    private double elevatorPreviousVelocity = 0.0;
    private double motorVoltage = 0.0;
    private double motorCurrent = 0.0;
    private double targetHeight = 0.0;

    private final VoltageOut m_voltageRequest = new VoltageOut(0).withUpdateFreqHz(0);
    private final MotionMagicVoltage m_motionMagicRequest = new MotionMagicVoltage(0).withUpdateFreqHz(0);

    public ElevatorSubsystem() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Feedback.SensorToMechanismRatio = ElevatorConstants.kGearRatio;
        config.MotorOutput.NeutralMode = ElevatorConstants.kNeutralMode;

        config.CurrentLimits.SupplyCurrentLimit = ElevatorConstants.kSupplyCurrentLimitAmps;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = ElevatorConstants.kStatorCurrentLimitAmps;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        config.MotionMagic.MotionMagicCruiseVelocity = ElevatorConstants.kCruiseVelocityRotPerSec;
        config.MotionMagic.MotionMagicAcceleration = ElevatorConstants.kCruiseAccelerationRotPerSecSq;

        config.Slot0.kP = ElevatorConstants.kP;
        config.Slot0.kI = ElevatorConstants.kI;
        config.Slot0.kD = ElevatorConstants.kD;
        config.Slot0.kS = ElevatorConstants.kS;
        config.Slot0.kG = ElevatorConstants.kG;
        config.Slot0.kV = ElevatorConstants.kV;
        config.Slot0.kA = ElevatorConstants.kA;

        m_leader.getConfigurator().apply(config);
        m_follower.getConfigurator().apply(config);

        m_follower.setControl(new Follower(
                ElevatorConstants.kLeaderMotorID,
                ElevatorConstants.kFollowerOpposed
                        ? MotorAlignmentValue.Opposed
                        : MotorAlignmentValue.Aligned));
    }

    @Override
    public void periodic() {
        elevatorHeight =  motorRotationToElevatorHeight(m_leader.getPosition().getValueAsDouble());
        elevatorVelocity =  motorRotationToElevatorHeight(m_leader.getVelocity().getValueAsDouble());
        motorVoltage = m_leader.getMotorVoltage().getValueAsDouble();
        motorCurrent = m_leader.getStatorCurrent().getValueAsDouble();

        elevatorAcceleration = (elevatorVelocity - elevatorPreviousVelocity) / 0.020;
        elevatorPreviousVelocity = elevatorVelocity;
        switch(elevatorMode) {
            case NORMAL:
                m_leader.setControl(m_motionMagicRequest.withPosition(
                targetHeight / ElevatorConstants.kInchesPerMotorRotation));
                break;
            case MANUAL:
                break;
        }
        



        SmartDashboard.putNumber("Elevator/ElevatorHeight(inches)", elevatorHeight);
        SmartDashboard.putNumber("Elevator/ElevatorVelocity(inches per sec)", elevatorVelocity);
        SmartDashboard.putNumber("Elevator/ElevatorAcceleration(inches per sec^2)", elevatorAcceleration);
        SmartDashboard.putNumber("Elevator/MotorVoltage", motorVoltage);
        SmartDashboard.putNumber("Elevator/MotorCurrent", motorCurrent);

    }
    @Override
    public void simulationPeriodic() {
        m_leaderSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        m_followerSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        double motorVoltage = m_leaderSim.getMotorVoltage();
        m_elevatorSim.setInputVoltage(motorVoltage);
        m_elevatorSim.update(.020);
        elevatorHeight = Units.metersToInches(m_elevatorSim.getPositionMeters());            
        elevatorVelocity = Units.metersToInches(m_elevatorSim.getVelocityMetersPerSecond());

        m_leaderSim.setRawRotorPosition(elevatorHeightToMotorRotation(Units.metersToInches(m_elevatorSim.getPositionMeters())  ) * ElevatorConstants.kGearRatio);
        m_leaderSim.setRotorVelocity(elevatorHeightToMotorRotation(Units.metersToInches(m_elevatorSim.getVelocityMetersPerSecond())) * ElevatorConstants.kGearRatio);
        m_leaderSim.setRotorAcceleration(elevatorHeightToMotorRotation(elevatorAcceleration));
        m_followerSim.setRawRotorPosition(-elevatorHeightToMotorRotation(elevatorHeight) * ElevatorConstants.kGearRatio);
        m_followerSim.setRotorVelocity(-elevatorHeightToMotorRotation(elevatorVelocity) * ElevatorConstants.kGearRatio);
        m_followerSim.setRotorAcceleration(-elevatorHeightToMotorRotation(elevatorAcceleration));
    }

    public Command manualJogCommand(double volts) {

        return run(() -> {
            elevatorMode = ElevatorMode.MANUAL;
            m_leader.setControl(m_voltageRequest.withOutput(volts));
        }).finallyDo(() ->{
            targetHeight = elevatorHeight;
            elevatorMode = ElevatorMode.NORMAL;
        });

    }

    public Command stopCommand() {
        return runOnce(this::stopMotors);
    }
    public void stopMotors() {
        m_leader.setControl(new NeutralOut());
    }

    public Command homeCommand() {
        final VoltageOut voltageRequest = new VoltageOut(0).withUpdateFreqHz(0);
        Timer debounceTimer = new Timer();
        return run(() -> m_leader.setControl(voltageRequest.withOutput(ElevatorConstants.kHomingVoltage)))
                .beforeStarting(debounceTimer::reset)
                .until(() -> {
                    boolean stopped = Math.abs(elevatorVelocity) < ElevatorConstants.kHomingVelocityThreshold;
                    if (!stopped) {
                        debounceTimer.reset();
                        return false;
                    }
                    return debounceTimer.hasElapsed(ElevatorConstants.kHomingDebounceSeconds);
                })
                .andThen(runOnce(() -> {
                    m_leader.setControl(voltageRequest.withOutput(0));
                    m_leader.setPosition(0);
                }));
    }

    public double elevatorHeightToMotorRotation(double elevatorHeight) {
        return elevatorHeight * (1.0/ElevatorConstants.kPulleyCircumferenceInches) * (1.0/ElevatorConstants.kCascadeFactor);
    }


    public double motorRotationToElevatorHeight(double motorRotation) {
        return motorRotation * ElevatorConstants.kPulleyCircumferenceInches * ElevatorConstants.kCascadeFactor; 
    }

    public void goToStowed() {
        targetHeight = ElevatorConstants.kStowedHeightInches;
    }

    public void goToL1() {
        targetHeight = ElevatorConstants.kL1HeightInches;
    }

    public void goToL2() {
        targetHeight = ElevatorConstants.kL2HeightInches;
    }

    public void goToL3() {
        targetHeight = ElevatorConstants.kL3HeightInches;
    }

    public boolean atGoal() {
        return Math.abs(elevatorHeight - targetHeight) < ElevatorConstants.kPositionToleranceInches;
    }

    public Command zeroEncoderCommand() {
        return runOnce(() -> m_leader.setPosition(0));
    }

    public double getElevatorHeight() {
        return elevatorHeight;
    }

    public double getElevatorVelocity() {
        return elevatorVelocity;
    }
}