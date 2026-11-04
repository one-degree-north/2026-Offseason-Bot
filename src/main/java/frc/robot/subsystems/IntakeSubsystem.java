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
import frc.robot.constants.subsystems.IndexerConstants;
import frc.robot.constants.subsystems.IntakeConstants;
import frc.robot.sensors.BeamBreakSensor;

public class IntakeSubsystem extends SubsystemBase {
    // Motor and sensors
    private final ___ m_motor = ___ // TODO: initialize a new TalonFX motor
    private final ___ m_beamBreak = // TODO: initialize a new BeamBreakSensor (tip: check the `sensors/` directory)

    // Simulation
    private final TalonFXSimState m_motorSim = m_motor.getSimState();
    private final DCMotorSim m_plant = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), IntakeConstants.MOI,
                    IntakeConstants.GEAR_RATIO),
            DCMotor.getKrakenX60(1));

    // Control request
    private final ___ m_voltageOut = new ___ // TODO: initialize a new VoltageOut control request

    public IntakeSubsystem() {
        var cfg = new TalonFXConfiguration();
        // TODO: fill in the value from the constants file
        cfg.Feedback.SensorToMechanismRatio = 
        cfg.MotorOutput.NeutralMode = 
        cfg.CurrentLimits.StatorCurrentLimit = 
        cfg.CurrentLimits.SupplyCurrentLimit = 
        cfg.CurrentLimits.SupplyCurrentLowerLimit = 
        cfg.CurrentLimits.SupplyCurrentLowerTime = 
        cfg.CurrentLimits.StatorCurrentLimitEnable = 
        cfg.CurrentLimits.SupplyCurrentLimitEnable = 

        // TODO: apply the configuration
    }

    public void runAtVolts(double volts) {
        // TODO: apply the control
    }

    public Command runAtVoltsCommand(double volts) {
        return runEnd(() -> {
            // TODO: run the motor at the given volts (tip: look at the method above)
        }, /* TODO: stop the motor */).withName("Intake running at " + volts + " volts");
    }

    public void stop() {
        // TODO: disable the motor (one line)
        m_motor.disable();
    }

    public boolean hasGamePiece() {
        // TODO: return true if the beam break is broken, and false otherwise (one line)
    }

    @Override
    public void periodic() {
        // Log the motor's stator current, supply current, motor voltage, and velocity
        SmartDashboard.putNumber("Intake/Motor/Stator current (A)", /* TODO: fill in */);
        SmartDashboard.putNumber("Intake/Motor/Supply current (A)", * TODO: fill in */);
        SmartDashboard.putNumber("Intake/Motor/Applied voltage (V)", * TODO: fill in */);
        SmartDashboard.putNumber("Intake/Motor/Velocity (mechanism RPS)", * TODO: fill in */);
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
                        Units.radiansToRotations(m_plant.getAngularVelocityRadPerSec())
                                * IntakeConstants.GEAR_RATIO);
    }
}
