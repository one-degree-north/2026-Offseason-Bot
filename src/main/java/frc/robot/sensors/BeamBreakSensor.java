package frc.robot.sensors;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DIOSim;

public class BeamBreakSensor {
    private final DigitalInput m_beamBreak;
    private final Debouncer m_debouncer;
    private final boolean m_falseWhenBroken;
    private DIOSim m_beamBreakSim;

    private boolean m_isBroken = false;

    public BeamBreakSensor(int channelId, double debounceSeconds, boolean falseWhenBroken) {
        m_beamBreak = new DigitalInput(channelId);
        m_debouncer = new Debouncer(debounceSeconds, DebounceType.kBoth);
        m_falseWhenBroken = falseWhenBroken;

        if (!RobotBase.isReal()) {
            m_beamBreakSim = new DIOSim(m_beamBreak);
        }
    }

    public void update() {
        boolean debounced = m_debouncer.calculate(m_beamBreak.get());
        if (m_falseWhenBroken) {
            m_isBroken = !debounced;
        } else {
            m_isBroken = debounced;
        }
    }

    public boolean isBroken() {
        return m_isBroken;
    }

    public void setSimBroken(boolean isBroken) {
        if (m_beamBreakSim == null) {
            return;
        }

        if (isBroken) {
            m_beamBreakSim.setValue(!m_falseWhenBroken);
        } else {
            m_beamBreakSim.setValue(m_falseWhenBroken);
        }
    }
}
