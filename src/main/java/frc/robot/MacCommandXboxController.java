package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * <p>
 * A custom CommandXboxController wrapper for Mac
 * 
 * Methods safe to use on Mac:
 * 
 * <ul>
 * <li><code>a()</code></li>
 * <li><code>b()</code></li>
 * <li><code>x()</code></li>
 * <li><code>y()</code></li>
 * <li><code>leftBumper()</code></li>
 * <li><code>rightBumper()</code></li>
 * <li><code>back()</code></li>
 * <li><code>start()</code></li>
 * <li><code>leftStick()</code></li>
 * <li><code>rightStick()</code></li>
 * <li><code>leftTrigger(double threshold)</code></li>
 * <li><code>leftTrigger()</code></li>
 * <li><code>rightTrigger(double threshold)</code></li>
 * <li><code>rightTrigger()</code></li>
 * <li><code>getLeftX()</code></li>
 * <li><code>getRightX()</code></li>
 * <li><code>getLeftY()</code></li>
 * <li><code>getRightY()</code></li>
 * <li><code>getLeftTriggerAxis()</code></li>
 * <li><code>getRightTriggerAxis()</code></li>
 * </p>
 */
public class MacCommandXboxController extends CommandXboxController {
    private final double defaultAxisThreshold = 0.5;

    public enum Button {
        /** A button. */
        kA(1),
        /** B button. */
        kB(2),
        /** X button. */
        kX(4),
        /** Y button. */
        kY(5),
        /** Left bumper button. */
        kLeftBumper(7),
        /** Right bumper button. */
        kRightBumper(8),
        /** Back button. */
        kBack(11),
        /** Start button. */
        kStart(12),
        /** Left stick button. */
        kLeftStick(14),
        /** Right stick button. */
        kRightStick(15);

        public final int value;

        Button(int value) {
            this.value = value;
        }
    }

    public enum Axis {
        /** Left X axis. */
        kLeftX(0),
        /** Right X axis. */
        kRightX(2),
        /** Left Y axis. */
        kLeftY(1),
        /** Right Y axis. */
        kRightY(3),
        /** Left trigger. */
        kLeftTrigger(5),
        /** Right trigger. */
        kRightTrigger(4);

        public final int value;

        Axis(int value) {
            this.value = value;
        }
    }

    public MacCommandXboxController(int port) {
        super(port);
    }

    @Override
    public Trigger a() {
        return button(Button.kA.value);
    }

    @Override
    public Trigger b() {
        return button(Button.kB.value);
    }

    @Override
    public Trigger x() {
        return button(Button.kX.value);
    }

    @Override
    public Trigger y() {
        return button(Button.kY.value);
    }

    @Override
    public Trigger leftBumper() {
        return button(Button.kLeftBumper.value);
    }

    @Override
    public Trigger rightBumper() {
        return button(Button.kRightBumper.value);
    }

    @Override
    public Trigger back() {
        return button(Button.kBack.value);
    }

    @Override
    public Trigger start() {
        return button(Button.kStart.value);
    }

    @Override
    public Trigger leftStick() {
        return button(Button.kLeftStick.value);
    }

    @Override
    public Trigger rightStick() {
        return button(Button.kRightStick.value);
    }

    @Override
    public Trigger leftTrigger(double threshold) {
        return new Trigger(() -> getLeftTriggerAxis() > threshold);
    }

    @Override
    public Trigger leftTrigger() {
        return new Trigger(() -> getLeftTriggerAxis() > defaultAxisThreshold);
    }

    @Override
    public Trigger rightTrigger(double threshold) {
        return new Trigger(() -> getRightTriggerAxis() > threshold);
    }

    @Override
    public Trigger rightTrigger() {
        return new Trigger(() -> getRightTriggerAxis() > defaultAxisThreshold);
    }

    @Override
    public double getLeftX() {
        return getRawAxis(Axis.kLeftX.value);
    }

    @Override
    public double getRightX() {
        return getRawAxis(Axis.kRightX.value);
    }

    @Override
    public double getLeftY() {
        return getRawAxis(Axis.kLeftY.value);
    }

    @Override
    public double getRightY() {
        return getRawAxis(Axis.kRightY.value);
    }

    @Override
    public double getLeftTriggerAxis() {
        // Normalize value
        return (getRawAxis(Axis.kLeftTrigger.value) + 1) / 2;
    }

    @Override
    public double getRightTriggerAxis() {
        // Normalize value
        return (getRawAxis(Axis.kRightTrigger.value) + 1) / 2;
    }
}
