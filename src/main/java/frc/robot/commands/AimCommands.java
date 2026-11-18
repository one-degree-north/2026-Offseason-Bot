package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ConeClashField;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.DrivetrainSubsystem;

public final class AimCommands {
    private AimCommands() {
    }

    public static Command aimAtGoalCommand(DrivetrainSubsystem drivetrain, DoubleSupplier xInput,
            DoubleSupplier yInput) {
        var request = new SwerveRequest.FieldCentricFacingAngle()
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        request.withHeadingPID(6.0, 0, 0);
        return drivetrain.applyRequest(() -> {
            var pose = drivetrain.getState().Pose;
            var goal = ConeClashField.AllianceFlip.forAlliance(FieldConstants.GOAL_CENTER);
            var fieldHeading = goal.minus(pose.getTranslation()).getAngle();
            var heading = fieldHeading.minus(drivetrain.getOperatorForwardDirection());
            return request.withVelocityX(xInput.getAsDouble()).withVelocityY(yInput.getAsDouble())
                    .withTargetDirection(heading);
        }).withName("AimAtGoal");
    }
}