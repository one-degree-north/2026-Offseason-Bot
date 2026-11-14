package frc.robot.subsystems;

import frc.robot.constants.subsystems.ElevatorConstants;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    private final TalonFX m_leader = new TalonFX(ElevatorConstants.kLeaderMotorID, ElevatorConstants.kCANBus);
    private final TalonFX m_follower = new TalonFX(ElevatorConstants.kFollowerMotorID, ElevatorConstants.kCANBus);

    private final TalonFXSimState m_leaderSim = m_leader.getSimState();
    private final TalonFXSimState m_followerSim = m_follower.getSimState();
    private final DCMotor m_gearbox = DCMotor.getKrakenX60(2);
    private final ElevatorSim m_elevatorSim = new ElevatorSim(m_gearbox, ElevatorConstants.kGearRatio,
            ElevatorConstants.kCarriageMass, ElevatorConstants.drumRadius * 0.0254, ElevatorConstants.kMinHeightInches * 0.0254,
            ElevatorConstants.kMaxHeightInches * 0.0254, true, 0);

    private double elevatorHeight = 0.0;
    private double elevatorVelocity = 0.0;
    private double elevatorAcceleration = 0.0;
    private double previousVelocity = 0.0;
    private double motorVoltage = 0.0;
    private double motorCurrent = 0.0;
    private double lastTargetHeight = 0.0;

    private final VoltageOut m_voltageRequest = new VoltageOut(0).withUpdateFreqHz(0);
    private final MotionMagicVoltage m_motionMagicRequest = new MotionMagicVoltage(0).withUpdateFreqHz(0);

    public ElevatorSubsystem() {
        TalonFXConfiguration config = new TalonFXConfiguration();

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
        elevatorHeight = m_leader.getPosition().getValueAsDouble() * ElevatorConstants.kInchesPerMotorRotation;
        elevatorVelocity = m_leader.getVelocity().getValueAsDouble() * ElevatorConstants.kInchesPerMotorRotation;
        motorVoltage = m_leader.getMotorVoltage().getValueAsDouble();
        motorCurrent = m_leader.getStatorCurrent().getValueAsDouble();

        elevatorAcceleration = (elevatorVelocity - previousVelocity) / 0.020;
        previousVelocity = elevatorVelocity;

        SmartDashboard.putNumber("Elevator/ElevatorHeight", elevatorHeight);
        SmartDashboard.putNumber("Elevator/ElevatorVelocity", elevatorVelocity);
        SmartDashboard.putNumber("Elevator/ElevatorAcceleration", elevatorAcceleration);
        SmartDashboard.putNumber("Elevator/MotorVoltage", motorVoltage);
        SmartDashboard.putNumber("Elevator/MotorCurrent", motorCurrent);
    }

    public void simulationPeriodic() {
        m_leaderSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        m_followerSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        double motorVoltage = m_leaderSim.getMotorVoltage();
        m_elevatorSim.setInputVoltage(motorVoltage);
        m_elevatorSim.update(.020);
        double positionRotations = m_elevatorSim.getPositionMeters()
        / ElevatorConstants.kInchesPerMotorRotation * 39.3701; 
        double velocityRotPerSec = m_elevatorSim.getVelocityMetersPerSecond()
        / ElevatorConstants.kInchesPerMotorRotation * 39.3701;

        m_leaderSim.setRawRotorPosition(positionRotations);
        m_leaderSim.setRotorVelocity(velocityRotPerSec);
    }

    public Command manualJogCommand() {
        return run(() -> m_leader.setControl(m_voltageRequest.withOutput(ElevatorConstants.kManualJogVolts)))
                .finallyDo(() -> m_leader.disable());
    }

    public Command stopCommand() {
        return runOnce(() -> m_leader.setControl(m_voltageRequest.withOutput(0)));
    }

    public Command goToHeightCommand(double heightInches) {
        return runOnce(() -> {
            lastTargetHeight = heightInches;
            m_leader.setControl(m_motionMagicRequest.withPosition(
                    heightInches / ElevatorConstants.kInchesPerMotorRotation));
        }).andThen(run(() -> {
        })).until(this::atGoal);
    }

    public Command goToStowed() {
        return goToHeightCommand(ElevatorConstants.kStowedHeightInches);
    }

    public Command goToL1() {
        return goToHeightCommand(ElevatorConstants.kL1HeightInches);
    }

    public Command goToL2() {
        return goToHeightCommand(ElevatorConstants.kL2HeightInches);
    }

    public Command goToL3() {
        return goToHeightCommand(ElevatorConstants.kL3HeightInches);
    }

    public boolean atGoal() {
        return Math.abs(elevatorHeight - lastTargetHeight) < ElevatorConstants.kPositionToleranceInches;
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