// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.constants.AutoConstants;
import frc.robot.constants.ControllerConstants.ControllerAxis;
import frc.robot.constants.ControllerConstants.ControllerBumpers;
import frc.robot.constants.ControllerConstants.ControllerRumble;
import frc.robot.constants.subsystems.IndexerConstants;
import frc.robot.constants.subsystems.IntakeConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final IntakeSubsystem m_intake = new IntakeSubsystem();
    private final IndexerSubsystem m_indexer = new IndexerSubsystem();

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
    }

    private void configureBindings() {
        // Low-priority trigger must exclude the high-priority trigger
        Trigger hasGamePiece = new Trigger(m_indexer::hasGamePiece);
        Trigger manualEjectButton = m_driverController.button(ControllerBumpers.RIGHT_BUMPER);
        Trigger manualIntakeButton = m_driverController.button(ControllerBumpers.LEFT_BUMPER)
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

        // Controller rumble
        new Trigger(m_indexer::hasGamePiece)
                .onTrue(rumblePulse());
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
                m_intake.runAtVoltsCommand(IntakeConstants.INTAKE_VOLTS)
                        .withTimeout(AutoConstants.IntakeAutoConstants.MAX_TIMEOUT_SECONDS));
    }
}
