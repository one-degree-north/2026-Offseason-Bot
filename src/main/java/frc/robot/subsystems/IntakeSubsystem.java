package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    private final TalonFX m_motor = new TalonFX(IntakeConstants.DEVICE_ID);

    private final TalonFXSimState m_motorSim = m_motor.getSimState();
    private final DCMotorSim m_plant = new DCMotorSim(
            LinearSystemId.createSingleJointedArmSystem(DCMotor.getKrakenX60(1), IntakeConstants.MOI,
                    IntakeConstants.GEAR_RATIO),
            DCMotor.getKrakenX60(1));

    private final VoltageOut m_voltageOut = new VoltageOut(0.0);

    public IntakeSubsystem() {
        var cfg = new TalonFXConfiguration();
        cfg.Feedback.SensorToMechanismRatio = IntakeConstants.GEAR_RATIO;
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        cfg.CurrentLimits.StatorCurrentLimit = IntakeConstants.STATOR_CURRENT_LIMIT;
        cfg.CurrentLimits.SupplyCurrentLimit = IntakeConstants.SUPPLY_CURRENT_LIMIT;
        cfg.CurrentLimits.SupplyCurrentLowerLimit = IntakeConstants.SUPPLY_CURRENT_LOWER_LIMIT;
        cfg.CurrentLimits.SupplyCurrentLowerTime = IntakeConstants.SUPPLY_CURRENT_LOWER_TIME;
        cfg.CurrentLimits.StatorCurrentLimitEnable = true;
        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        m_motor.getConfigurator().apply(cfg);
    }

    public void runAtVolts(double volts) {
        m_motor.setControl(m_voltageOut.withOutput(volts));
    }

    public Command runAtVoltsCommand(double volts) {
        return runEnd(() -> {
            runAtVolts(volts);
        }, this::stop).withName("Intake running at " + volts + " volts");
    }

    public void stop() {
        m_motor.disable();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake/Motor/Stator current (A)", m_motor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Intake/Motor/Supply current (A)", m_motor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Intake/Motor/Applied voltage (V)", m_motor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Intake/Motor/Velocity (mechanism RPS)", m_motor.getVelocity().getValueAsDouble());
    }

    @Override
    public void simulationPeriodic() {
        m_motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        m_plant.setInputVoltage(m_motorSim.getMotorVoltage());
        m_plant.update(0.020);

        m_motorSim.setRawRotorPosition(
                Units.radiansToRotations(m_plant.getAngularPositionRad()) * IntakeConstants.GEAR_RATIO);
        m_motorSim
                .setRotorVelocity(
                        Units.radiansToRotations(m_plant.getAngularAccelerationRadPerSecSq()) * IntakeConstants.GEAR_RATIO);
    }
}

