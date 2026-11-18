# Flywheel Task

Build a flywheel shooter subsystem in simulation with one TalonFX Kraken X60 motor.

**Files**

- `subsystems/FlywheelSubsystem.java`: Most of your work
- `constants/subsystems/FlywheelConstants.java`: Given, but you will change `KP`

**Reference code:** `IntakeSubsystem`, `RobotContainer`, and the
[Phoenix 6 docs](https://v6.docs.ctr-electronics.com/) / [WPILib docs](https://docs.wpilib.org/).

## Rules

1. Do one level at a time.
2. Commit after each level (`flywheel: level 1 done`, etc.).

## Level 1: Spin it (open loop)

Make the motor spin at a fixed voltage.

TODOs: `VoltageOut`, the motor config, `runAtVolts`, `runAtVoltsCommand`, `stop`,
`getVelocityRps`, Level 1 logging. Then in `RobotContainer`, create the flywheel and bind
`rightBumper` (`R1`) to `runAtVoltsCommand(TEST_VOLTS)` while held.

**Done when**

- [ ] Sim runs, and holding the right bumper makes `Flywheel/Velocity (rps)` rise and then level off.
- [ ] It levels off at about **50 rps** at 6 V.
- [ ] When you let go, velocity coasts down slowly rather than snapping to 0.

## Level 2: Hold a speed (velocity control)

Use the motor controller's onboard PID to hold a target speed.

TODOs: `theoreticalKV`, the Slot0 gains, `VelocityVoltage`, `setTargetRps`, `runAtRpsCommand`,
target logging. Bind `povDown` to `runAtRpsCommand(TEST_TARGET_RPS)` while held.

**Done when**

- [ ] Holding rightTrigger (R2), AdvantageScope shows velocity reaching 60 rps and staying within ±0.5 rps.
- [ ] **Experiment:** temporarily multiply kV by 0.8 in the config. With `KP = 0`, how far off does
      it settle? Raise `KP` until the error is gone _without_ oscillating. Put kV back, keep your
      `KP`, and write both numbers in your commit message.

## Level 3: Shoot only when ready

TODOs: `atSpeed` and Level 3 logging.

**Done when**

- [ ] `Flywheel/At speed` is false while spinning up, true once at 60 rps, and false when stopped.
