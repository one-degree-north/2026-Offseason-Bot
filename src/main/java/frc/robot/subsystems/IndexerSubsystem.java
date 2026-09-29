// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.subsystems.IndexerConstants;
import frc.robot.sensors.BeamBreakSensor;

public class IndexerSubsystem extends SubsystemBase {
    private final TalonFX m_motor = new TalonFX(IndexerConstants.DEVICE_ID);
    private final BeamBreakSensor m_beamBreak = new BeamBreakSensor(IndexerConstants.BEAM_BREAK_CHANNEL,
            IndexerConstants.BEAM_BREAK_DEBOUNCE_SECONDS,
            IndexerConstants.BEAM_BREAK_FALSE_WHEN_BROKEN);

    private final TalonFXSimState m_motorSim = m_motor.getSimState();
    private final DCMotorSim m_plant = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1),
                    IndexerConstants.MOI,
                    IndexerConstants.GEAR_RATIO),
            DCMotor.getKrakenX60(1));

    private final VoltageOut m_voltageOut = new VoltageOut(0.0);

    public IndexerSubsystem() {
        var cfg = new TalonFXConfiguration();
        cfg.Feedback.SensorToMechanismRatio = IndexerConstants.GEAR_RATIO;
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        cfg.CurrentLimits.StatorCurrentLimit = IndexerConstants.STATOR_CURRENT_LIMIT;
        cfg.CurrentLimits.SupplyCurrentLimit = IndexerConstants.SUPPLY_CURRENT_LIMIT;
        cfg.CurrentLimits.SupplyCurrentLowerLimit = IndexerConstants.SUPPLY_CURRENT_LOWER_LIMIT;
        cfg.CurrentLimits.SupplyCurrentLowerTime = IndexerConstants.SUPPLY_CURRENT_LOWER_TIME;
        cfg.CurrentLimits.StatorCurrentLimitEnable = true; // defaults to true, but be explicit
        cfg.CurrentLimits.SupplyCurrentLimitEnable = true; // defaults to true, but be explicit
        m_motor.getConfigurator().apply(cfg);

        if (!RobotBase.isReal()) {
            SmartDashboard.putBoolean("Indexer/BeamBreak/Is broken", false);
        }
    }

    public void runAtVolts(double volts) {
        m_motor.setControl(m_voltageOut.withOutput(volts));
    }

    public Command runAtVoltsCommand(double volts) {
        return runEnd(() -> runAtVolts(volts), this::stop).withName("Indexer running at " + volts + " volts");
    }

    public void stop() {
        m_motor.disable();
    }

    public boolean hasGamePiece() {
        return m_beamBreak.isBroken();
    }

    public Command intakeCommand() {
        return runEnd(() -> runAtVolts(IndexerConstants.INDEX_VOLTS), this::stop).until(this::hasGamePiece)
                .withName("Indexer intaking");
    }

    public Command feedCommand() {
        return Commands.sequence(
                run(() -> runAtVolts(IndexerConstants.FEED_VOLTS)).until(() -> !hasGamePiece()),
                run(() -> runAtVolts(IndexerConstants.FEED_VOLTS))
                        .withTimeout(IndexerConstants.FEED_CLEAR_SECONDS))
                .finallyDo(this::stop)
                .withName("Indexer feeding");
    }

    public Command ejectCommand() {
        return runEnd(() -> runAtVolts(IndexerConstants.EJECT_VOLTS), this::stop).withName("Indexer ejecting");
    }

    @Override
    public void periodic() {
        m_beamBreak.update();

        SmartDashboard.putNumber("Indexer/Motor/Stator current (A)",
                m_motor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Indexer/Motor/Supply current (A)",
                m_motor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Indexer/Motor/Applied voltage (V)",
                m_motor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Indexer/Motor/Velocity (mechanism RPS)",
                m_motor.getVelocity().getValueAsDouble());
        SmartDashboard.putBoolean("Indexer/BeamBreak/Has game piece", hasGamePiece());
    }

    @Override
    public void simulationPeriodic() {
        m_motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        m_plant.setInputVoltage(m_motorSim.getMotorVoltage());
        m_plant.update(0.020);

        m_motorSim.setRawRotorPosition(
                Units.radiansToRotations(m_plant.getAngularPositionRad()) *
                        IndexerConstants.GEAR_RATIO);
        m_motorSim
                .setRotorVelocity(
                        Units.radiansToRotations(m_plant.getAngularVelocityRadPerSec()) *
                                IndexerConstants.GEAR_RATIO);

        boolean simBeamBreakBroken = SmartDashboard.getBoolean("Indexer/BeamBreak/Is broken", false);
        m_beamBreak.setSimBroken(simBeamBreakBroken);
    }
}
