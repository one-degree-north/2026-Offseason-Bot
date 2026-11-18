package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.subsystems.FlywheelConstants;

/**
 * Flywheel shooter, driven by one Kraken X60.
 *
 * Work through FLYWHEEL_TASK.md level by level. Each TODO is labelled.
 */
public class FlywheelSubsystem extends SubsystemBase {
    // Motor (given)
    private final TalonFX m_motor = new TalonFX(FlywheelConstants.DEVICE_ID);

    // Control requests
    private final VoltageOut m_voltageOut = null; // TODO: (Level 1): create the request
    private final VelocityVoltage m_velocityRequest = null; // TODO: (Level 2): create the request, using slot 0

    // The speed we are currently asking for, in rotations per second. 0 = not
    // trying to hold a speed.
    private double m_targetRps = 0.0;

    // Simulation (given, no need to change)
    private final TalonFXSimState m_motorSim = m_motor.getSimState();
    private final FlywheelSim m_plant = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60(1), FlywheelConstants.MOI,
                    FlywheelConstants.GEAR_RATIO),
            DCMotor.getKrakenX60(1));

    public FlywheelSubsystem() {
        var cfg = new TalonFXConfiguration();

        // TODO: (Level 1): set the gear ratio, neutral mode, and all current limits.

        // TODO: (Level 2): set the Slot0 gains: kS, kP, kA from FlywheelConstants,
        // and kV from theoreticalKV(...)

        // TODO: (Level 1): apply the configuration
    }

    // ===================== Level 1: open loop =====================

    /** Runs the motor at a fixed voltage. No feedback. */
    public void runAtVolts(double volts) {
        // TODO: (Level 1). Also reset m_targetRps, since we are no longer holding a
        // speed.
    }

    /** While scheduled, run at the given voltage. Stop when it ends. */
    public Command runAtVoltsCommand(double volts) {
        return Commands.none(); // TODO: (Level 1)
    }

    /** Stops the motor (NeutralOut) and clears the target. */
    public void stop() {
        // TODO: (Level 1)
    }

    /** Current flywheel speed in rotations per second (flywheel, not motor). */
    public double getVelocityRps() {
        return 0.0; // TODO: (Level 1)
    }

    // ===================== Level 2: velocity control =====================

    /**
     * Theoretical kV in **volts per (flywheel rotation per second)**.
     *
     * At 12 V with no load, the motor spins at its free speed. How fast does the
     * flywheel spin then?
     */
    public static double theoreticalKV(double gearRatio) {
        return 0.0; // TODO: (Level 2)
    }

    /** Tells the motor controller to hold this speed, using its onboard PID. */
    public void setTargetRps(double rps) {
        // TODO: (Level 2): remember the target, then send the velocity request
    }

    /**
     * While scheduled, hold the given speed. Stop when it ends. Call setTargetRps.
     */
    public Command runAtRpsCommand(double rps) {
        return Commands.none(); // TODO: (Level 2)
    }

    // ===================== Level 3: ready to shoot? =====================

    /**
     * True if we are holding a speed and are within TOLERANCE_RPS of it.
     */
    public boolean atSpeed() {
        return false; // TODO:: (Level 3)
    }

    // ===================== Logging =====================

    @Override
    public void periodic() {
        // Use keys that start with "Flywheel/" so the values group together in
        // AdvantageScope.
        // TODO: (Level 1): log velocity (rps), applied voltage, and stator current,
        // TODO: (Level 2): also log the target (rps).
        // TODO: (Level 3): also log atSpeed().
    }

    @Override
    public void simulationPeriodic() {
        m_motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        m_plant.setInputVoltage(m_motorSim.getMotorVoltage());
        m_plant.update(0.020);

        double flywheelRps = Units.radiansToRotations(m_plant.getAngularVelocityRadPerSec());
        double rotorRps = flywheelRps * FlywheelConstants.GEAR_RATIO;
        m_motorSim.setRotorVelocity(rotorRps);
        m_motorSim.addRotorPosition(rotorRps * 0.020);
    }
}
