// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.constants.ControllerConstants.ControllerAxis;
import frc.robot.constants.ControllerConstants.ControllerButtons;
import frc.robot.constants.ControllerConstants.ControllerRumble;
import frc.robot.constants.subsystems.IndexerConstants;
import frc.robot.constants.subsystems.IntakeConstants;
import frc.robot.constants.subsystems.DrivetrainTunerConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class RobotContainer {
    private double MaxSpeed = 1.0 * DrivetrainTunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts
                                                                                                  // desired top
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.RobotCentric aim = new SwerveRequest.RobotCentric().withDeadband(0.0)
            .withRotationalDeadband(0.0).withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    // The robot's subsystems and commands are defined here...
    private final IntakeSubsystem m_intake = new IntakeSubsystem();
    private final IndexerSubsystem m_indexer = new IndexerSubsystem();
    public final DrivetrainSubsystem m_drivetrain = DrivetrainTunerConstants.createDrivetrain();

    private final Telemetry m_logger = new Telemetry(MaxSpeed);

    private final CommandXboxController m_driverController = new CommandXboxController(0);

    public RobotContainer() {
        // Configure the trigger bindings
        configureBindings();

        // Configure the default commands
        configureDefaultCommands();
    }

    private void configureDefaultCommands() {
        m_intake.setDefaultCommand(m_intake.run(() -> {
            if (m_indexer.hasGamePiece()) {
                m_intake.runAtVolts(IntakeConstants.HOLD_VOLTS);
            } else {
                m_intake.stop();
            }
        }));

        m_drivetrain.setDefaultCommand(m_drivetrain.applyRequest(
                () -> drive.withVelocityX(-m_driverController.getRawAxis(ControllerAxis.LEFT_Y) * MaxSpeed)
                        .withVelocityY(-m_driverController.getRawAxis(ControllerAxis.LEFT_X) * MaxSpeed)
                        .withRotationalRate(
                                -m_driverController.getRawAxis(ControllerAxis.RIGHT_X) * MaxAngularRate)));
    }

    private void configureBindings() {
        // Low-priority trigger must exclude the high-priority trigger
        Trigger hasGamePiece = new Trigger(m_indexer::hasGamePiece);
        Trigger manualEjectButton = m_driverController.button(ControllerButtons.RIGHT_BUMPER);
        Trigger manualIntakeButton = m_driverController.button(ControllerButtons.LEFT_BUMPER)
                .and(manualEjectButton.negate());
        Trigger smartIntakeRequest = m_driverController
                .axisGreaterThan(ControllerAxis.LEFT_TRIGGER, 0.5)
                .and(manualIntakeButton.negate())
                .and(manualEjectButton.negate())
                .and(hasGamePiece.negate());

        // Smart intake
        smartIntakeRequest
                .whileTrue(m_indexer.intakeCommand()
                        .deadlineFor(m_intake.runAtVoltsCommand(IntakeConstants.INTAKE_VOLTS)));

        // Manual intake
        manualIntakeButton
                .whileTrue(m_intake.runAtVoltsCommand(IntakeConstants.INTAKE_VOLTS)
                        .alongWith(m_indexer.runAtVoltsCommand(IndexerConstants.INDEX_VOLTS)));

        // Manual eject
        manualEjectButton
                .whileTrue(m_intake.runAtVoltsCommand(IntakeConstants.EJECT_VOLTS).alongWith(m_indexer.ejectCommand()));

        // Face the bot toward the AprilTag
        m_driverController.axisGreaterThan(ControllerAxis.RIGHT_TRIGGER, 0.5)
                .whileTrue(m_drivetrain.applyRequest(() -> {
                    if (!LimelightHelpers.getTV("limelight"))
                        return aim.withVelocityX(0.0)
                                .withVelocityY(0.0)
                                .withRotationalRate(0.0);
                    double fwd = MathUtil.clamp(LimelightHelpers.getTY("limelight") * -0.1, -1.0, 1.0);
                    double rot = MathUtil.clamp(LimelightHelpers.getTX("limelight") * -0.05, -MaxAngularRate,
                            MaxAngularRate);
                    return aim.withVelocityX(fwd).withVelocityY(0.0).withRotationalRate(rot);
                }));

        // Controller rumble
        hasGamePiece.onTrue(rumblePulse());

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                m_drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        m_driverController.button(ControllerButtons.A).whileTrue(m_drivetrain.applyRequest(() -> brake));
        m_driverController.button(ControllerButtons.B).whileTrue(m_drivetrain.applyRequest(() -> point
                .withModuleDirection(new Rotation2d(-m_driverController.getRawAxis(ControllerAxis.LEFT_Y),
                        -m_driverController.getRawAxis(ControllerAxis.LEFT_X)))));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        m_driverController.button(ControllerButtons.BACK).and(m_driverController.button(ControllerButtons.Y))
                .whileTrue(m_drivetrain.sysIdDynamic(Direction.kForward));
        m_driverController.button(ControllerButtons.BACK).and(m_driverController.button(ControllerButtons.X))
                .whileTrue(m_drivetrain.sysIdDynamic(Direction.kReverse));
        m_driverController.button(ControllerButtons.START).and(m_driverController.button(ControllerButtons.Y))
                .whileTrue(m_drivetrain.sysIdQuasistatic(Direction.kForward));
        m_driverController.button(ControllerButtons.START).and(m_driverController.button(ControllerButtons.X))
                .whileTrue(m_drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on the right joystick press.
        m_driverController.button(ControllerButtons.RIGHT_JOYSTICK)
                .onTrue(m_drivetrain.runOnce(m_drivetrain::seedFieldCentric));

        /* Register the swerve telemetry */
        m_drivetrain.registerTelemetry(m_logger::telemeterize);
    }

    private void setDriverControllerRumble(double intensity) {
        m_driverController.setRumble(RumbleType.kLeftRumble, intensity);
        m_driverController.setRumble(RumbleType.kRightRumble, intensity);
    }

    private Command rumblePulse() {
        return Commands.startEnd(
                () -> setDriverControllerRumble(ControllerRumble.INTENSITY),
                () -> setDriverControllerRumble(0.0))
                .withTimeout(ControllerRumble.DURATION_SECONDS)
                .withName("Driver rumble pulse");
    }

    public void stopDriverControllerRumble() {
        setDriverControllerRumble(0.0);
    }

    public Command getAutonomousCommand() {
        return Commands.sequence(
                m_drivetrain.runOnce(() -> {
                    m_drivetrain.resetPose(new Pose2d(0, 0, new Rotation2d()));
                }));
    }
}
