package frc.robot.commandCreators;

import java.util.function.Function;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;

public class TelemetryCommands {

        public static final class Constants {
                public static final Pose2d TwoNoteTopStart = new Pose2d(0.83, 6.49, Rotation2d.fromDegrees(58.50));
                public static final Pose2d TwoNoteMidStart = new Pose2d(1.22, 5.50, Rotation2d.fromDegrees(0));
                public static final Pose2d TwoNoteMidBottom = new Pose2d(0.68, 4.50, Rotation2d.fromDegrees(-59.62));
                public static final Pose2d AmpStart = new Pose2d(1.56, 7.41, Rotation2d.fromDegrees(90));

                public static final double fieldLengthMeters = 16.542;

        }

        public static final Function<Pose2d, Command> createSetStartPoseCommand = (pose) -> {
                Runnable setPose = () -> {
                        if (DriverStation.getAlliance().isPresent()
                                        && DriverStation.getAlliance().get() == Alliance.Red) {
                                Rotation2d robotPoseRotation = pose.getRotation();
                                double degrees = robotPoseRotation.getDegrees();
                                degrees = -(degrees - 90) + 90;
                                robotPoseRotation = Rotation2d.fromDegrees(degrees);
                                Pose2d newPose = new Pose2d(Constants.fieldLengthMeters - pose.getX(), pose.getY(),
                                                robotPoseRotation);
                                RobotContainer.telemetrySubsystem.setPoseEstimator.accept(newPose);
                        } else {
                                RobotContainer.telemetrySubsystem.setPoseEstimator.accept(pose);
                        }
                };

                Command command = Commands.runOnce(setPose,
                                RobotContainer.telemetrySubsystem);
                command.setName("Set Pose");
                return command;
        };

        public TelemetryCommands() {

        }
}
