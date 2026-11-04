package frc.robot.constants;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.units.Volts;


public class ElevatorSubsystems {
    private final TalonFX m_lead_motor = new TalonFX(5);
    private final TalonFX m_follow_motor = new TalonFX(6);
    private final VoltageOut m_voltageRequest = new VoltageOut(0);
    public void setLeaderVoltage(double VoltageOut){
        
    }


}
 