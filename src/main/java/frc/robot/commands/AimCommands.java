package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ConeClashField;
import frc.robot.constants.AimConstants;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.DrivetrainSubsystem;

public final class AimCommands {
    private AimCommands() {
    }

    public static Command aimAtGoalCommand(DrivetrainSubsystem drivetrain, DoubleSupplier xInput,
            DoubleSupplier yInput) {
        var request = new SwerveRequest.FieldCentricFacingAngle()
                .withDriveRequestType(DriveRequestType.Velocity);
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

    public static Command aimAtGoalNoLatencyCommand(DrivetrainSubsystem drivetrain, DoubleSupplier xInput,
            DoubleSupplier yInput, Pose2d goalPose) {
        var request = new SwerveRequest.FieldCentricFacingAngle()
                .withDriveRequestType(DriveRequestType.Velocity);
        request.withHeadingPID(6.0, 0, 0);

        return drivetrain.applyRequest(() -> {
            SwerveDriveState driveState = drivetrain.getState();
            Translation2d goal = goalPose.getTranslation();
            double odometryLatency = Utils.getCurrentTimeSeconds() - driveState.Timestamp;
            ChassisSpeeds speeds = driveState.Speeds;

            Pose2d robotPose = driveState.Pose
                    .transformBy(
                            new Transform2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, new Rotation2d())
                                    .times(odometryLatency));
            var fieldHeading = goal.minus(robotPose.getTranslation()).getAngle();
            var heading = fieldHeading.minus(drivetrain.getOperatorForwardDirection());
            return request.withVelocityX(xInput.getAsDouble()).withVelocityY(yInput.getAsDouble())
                    .withTargetDirection(heading);
        }).withName("AimAtGoalNoLatency");

    }
}