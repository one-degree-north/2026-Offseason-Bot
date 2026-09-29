
package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.subsystems.SlapdownConstants;

public class SlapdownSubsystem extends SubsystemBase {
    private final TalonFX m_motor = new TalonFX(SlapdownConstants.DEVICE_ID);
    private final MotionMagicVoltage m_positionRequest = new MotionMagicVoltage(0.0);
    
    private double m_target = SlapdownConstants.STOWED_ROTATIONS;

    public SlapdownSubsystem() {
        var cfg = new TalonFXConfiguration();
        cfg.Feedback.SensorToMechanismRatio = SlapdownConstants.GEAR_RATIO;
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        cfg.CurrentLimits.StatorCurrentLimit = SlapdownConstants.STATOR_CURRENT_LIMIT;
        cfg.CurrentLimits.SupplyCurrentLimit = SlapdownConstants.SUPPLY_CURRENT_LIMIT;
        cfg.CurrentLimits.SupplyCurrentLowerLimit = SlapdownConstants.SUPPLY_CURRENT_LOWER_LIMIT;
        cfg.CurrentLimits.SupplyCurrentLowerTime = SlapdownConstants.SUPPLY_CURRENT_LOWER_TIME;
        cfg.CurrentLimits.StatorCurrentLimitEnable = true; // defaults to true, but be explicit
        cfg.CurrentLimits.SupplyCurrentLimitEnable = true; // defaults to true, but be explicit

        cfg.Slot0.kP = SlapdownConstants.KP;
        cfg.Slot0.kI = SlapdownConstants.KI;
        cfg.Slot0.kD = SlapdownConstants.KD;
        cfg.Slot0.kS = SlapdownConstants.KS;
        cfg.Slot0.kV = SlapdownConstants.KV;
        cfg.Slot0.kA = SlapdownConstants.KA;

        cfg.Slot0.kG = SlapdownConstants.KG;
        cfg.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        cfg.MotionMagic.MotionMagicCruiseVelocity = SlapdownConstants.MOTION_MAGIC_CRUISE_VELOCITY;
        cfg.MotionMagic.MotionMagicAcceleration = SlapdownConstants.MOTION_MAGIC_ACCELERATION;

        //TODO: add soft limits

        m_motor.getConfigurator().apply(cfg);

        m_motor.setPosition(SlapdownConstants.STOWED_ROTATIONS);
    }

    public Command goToPositionCommand(double rotations) {
        return run(() -> setTarget(rotations))
            .until(this::atTarget)
            .withName("Slapdown to " + rotations);
    }

    public void setTarget(double rotations) {
        m_target = rotations;
        m_motor.setControl(m_positionRequest.withPosition(rotations));
    }


    public boolean atTarget() {
        return Math.abs(m_motor.getPosition().getValueAsDouble() - m_target) < SlapdownConstants.TOLERANCE_ROTATIONS;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Slapdown/Motor/Stator current (A)",
                m_motor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Slapdown/Motor/Supply current (A)",
                m_motor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Slapdown/Motor/Applied voltage (V)",
                m_motor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Slapdown/TargetPosition",
                m_target);
        SmartDashboard.putNumber("Slapdown/Position (rot)",
                m_motor.getPosition().getValueAsDouble());
    }

}
