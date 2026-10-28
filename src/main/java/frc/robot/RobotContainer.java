// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.constants.ControllerConstants.ControllerAxis;
import frc.robot.constants.ControllerConstants.ControllerButtons;
import frc.robot.constants.ControllerConstants.ControllerRumble;
import frc.robot.constants.subsystems.DrivetrainTunerConstants;
import frc.robot.constants.subsystems.IndexerConstants;
import frc.robot.constants.subsystems.IntakeConstants;
import frc.robot.constants.subsystems.VisionConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.vision.LimelightCamera;
import frc.robot.subsystems.vision.PhotonVisionCamera;
import frc.robot.subsystems.vision.Vision;

public class RobotContainer {
        private double MaxSpeed = 1.0 * DrivetrainTunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts
                                                                                                      // desired top
        private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per

        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive
                                                                                 // motors
        private final SwerveRequest.RobotCentric aim = new SwerveRequest.RobotCentric().withDeadband(0.0)
                        .withRotationalDeadband(0.0).withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

        private final SendableChooser<Command> autoChooser;

        // The robot's subsystems and commands are defined here...
        private final IntakeSubsystem intake = new IntakeSubsystem();
        private final IndexerSubsystem indexer = new IndexerSubsystem();
        public final DrivetrainSubsystem drivetrain = DrivetrainTunerConstants.createDrivetrain();

        private final Vision vision = new Vision(
                        drivetrain::addVisionMeasurement,
                        () -> Units.radiansToRotations(drivetrain.getState().Speeds.omegaRadiansPerSecond),
                        new PhotonVisionCamera(
                                        VisionConstants.CAMERA_1_NAME,
                                        VisionConstants.CAM_1_OFFSET,
                                        VisionConstants.CAM_1_TRUST_FACTOR),
                        new LimelightCamera(
                                        VisionConstants.CAMERA_2_NAME,
                                        VisionConstants.CAM_2_OFFSET,
                                        () -> drivetrain.getState().Pose.getRotation().getDegrees(),
                                        VisionConstants.CAM_2_TRUST_FACTOR));

        private final Telemetry logger = new Telemetry(MaxSpeed);

        private final CommandXboxController driverController = new CommandXboxController(0);

        public RobotContainer() {
                // Configure the trigger bindings
                configureBindings();

                // Configure the default commands
                configureDefaultCommands();

                autoChooser = AutoBuilder.buildAutoChooser();
                SmartDashboard.putData("Auto Chooser", autoChooser);

                if (RobotBase.isSimulation()) {
                        SmartDashboard.putData("Vision/Sim: Kidnap estimate",
                                        Commands.runOnce(() -> {
                                                if (simTruePose != null) {
                                                        drivetrain.resetPose(simTruePose.transformBy(
                                                                        new Transform2d(1.0, 0.5,
                                                                                        Rotation2d.fromDegrees(15))));

                                                }
                                        }).ignoringDisable(true));

                }
        }

        private void configureDefaultCommands() {
                intake.setDefaultCommand(intake.run(() -> {
                        if (indexer.hasGamePiece()) {
                                intake.runAtVolts(IntakeConstants.HOLD_VOLTS);
                        } else {
                                intake.stop();
                        }
                }));

                drivetrain.setDefaultCommand(drivetrain.applyRequest(
                                () -> drive.withVelocityX(
                                                -driverController.getRawAxis(ControllerAxis.LEFT_Y) * MaxSpeed)
                                                .withVelocityY(-driverController.getRawAxis(ControllerAxis.LEFT_X)
                                                                * MaxSpeed)
                                                .withRotationalRate(
                                                                -driverController.getRawAxis(ControllerAxis.RIGHT_X)
                                                                                * MaxAngularRate)));
        }

        private void configureBindings() {
                // Low-priority trigger must exclude the high-priority trigger
                Trigger hasGamePiece = new Trigger(indexer::hasGamePiece);
                Trigger manualEjectButton = driverController.button(ControllerButtons.RIGHT_BUMPER);
                Trigger manualIntakeButton = driverController.button(ControllerButtons.LEFT_BUMPER)
                                .and(manualEjectButton.negate());
                Trigger smartIntakeRequest = driverController
                                .axisGreaterThan(ControllerAxis.LEFT_TRIGGER, 0.5)
                                .and(manualIntakeButton.negate())
                                .and(manualEjectButton.negate())
                                .and(hasGamePiece.negate());

                // Smart intake
                smartIntakeRequest
                                .whileTrue(indexer.intakeCommand()
                                                .deadlineFor(intake.runAtVoltsCommand(IntakeConstants.INTAKE_VOLTS)));

                // Manual intake
                manualIntakeButton
                                .whileTrue(intake.runAtVoltsCommand(IntakeConstants.INTAKE_VOLTS)
                                                .alongWith(indexer.runAtVoltsCommand(IndexerConstants.INDEX_VOLTS)));

                // Manual eject
                manualEjectButton
                                .whileTrue(intake.runAtVoltsCommand(IntakeConstants.EJECT_VOLTS)
                                                .alongWith(indexer.ejectCommand()));

                // Face the bot toward the AprilTag
                driverController.axisGreaterThan(ControllerAxis.RIGHT_TRIGGER, 0.5)
                                .whileTrue(drivetrain.applyRequest(() -> {
                                        if (!LimelightHelpers.getTV(VisionConstants.CAMERA_2_NAME))
                                                return aim.withVelocityX(0.0)
                                                                .withVelocityY(0.0)
                                                                .withRotationalRate(0.0);
                                        double fwd = MathUtil.clamp(
                                                        LimelightHelpers.getTY(VisionConstants.CAMERA_2_NAME) * -0.1,
                                                        -1.0,
                                                        1.0);
                                        double rot = MathUtil.clamp(
                                                        LimelightHelpers.getTX(VisionConstants.CAMERA_2_NAME) * -0.05,
                                                        -MaxAngularRate,
                                                        MaxAngularRate);
                                        return aim.withVelocityX(fwd).withVelocityY(0.0).withRotationalRate(rot);
                                }));

                // Controller rumble
                hasGamePiece.onTrue(rumblePulse());

                // Idle while the robot is disabled. This ensures the configured
                // neutral mode is applied to the drive motors while disabled.
                final var idle = new SwerveRequest.Idle();
                RobotModeTriggers.disabled().whileTrue(
                                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

                driverController.button(ControllerButtons.A).whileTrue(drivetrain.applyRequest(() -> brake));
                driverController.button(ControllerButtons.B).whileTrue(drivetrain.applyRequest(() -> point
                                .withModuleDirection(new Rotation2d(-driverController.getRawAxis(ControllerAxis.LEFT_Y),
                                                -driverController.getRawAxis(ControllerAxis.LEFT_X)))));

                // Run SysId routines when holding back/start and X/Y.
                // Note that each routine should be run exactly once in a single log.
                driverController.button(ControllerButtons.BACK).and(driverController.button(ControllerButtons.Y))
                                .whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
                driverController.button(ControllerButtons.BACK).and(driverController.button(ControllerButtons.X))
                                .whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
                driverController.button(ControllerButtons.START).and(driverController.button(ControllerButtons.Y))
                                .whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
                driverController.button(ControllerButtons.START).and(driverController.button(ControllerButtons.X))
                                .whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

                // Reset the field-centric heading on the right joystick press.
                driverController.button(ControllerButtons.RIGHT_JOYSTICK)
                                .onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

                // Register the swerve telemetry
                drivetrain.registerTelemetry(logger::telemeterize);
        }

        private void setDriverControllerRumble(double intensity) {
                driverController.setRumble(RumbleType.kLeftRumble, intensity);
                driverController.setRumble(RumbleType.kRightRumble, intensity);
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
                return autoChooser.getSelected();
        }

        // RobotContainer.java
        private Pose2d simTruePose = null;
        private final StructPublisher<Pose2d> simTruePosePublisher = NetworkTableInstance.getDefault()
                        .getStructTopic("Vision/Sim/TruePose", Pose2d.struct).publish();

        /** Call from Robot.simulationPeriodic(). */
        public void simulationPeriodic() {
                var state = drivetrain.getState();
                if (simTruePose == null) {
                        simTruePose = state.Pose; // start where the estimate starts
                }

                // Move the true pose by how far the robot moved in the last 20 ms.
                double dt = 0.02;
                simTruePose = simTruePose.exp(new Twist2d(
                                state.Speeds.vxMetersPerSecond * dt,
                                state.Speeds.vyMetersPerSecond * dt,
                                state.Speeds.omegaRadiansPerSecond * dt));
                vision.updateSimulation(simTruePose);
                simTruePosePublisher.set(simTruePose);
        }
}
