package frc.robot.commands;

import com.pathplanner.lib.path.PathConstraints;

import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics.SwerveDriveWheelStates;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;

public class CommandBinder {

        private CommandBinder() {
        }

        public static final Consumer<Trigger> bindManualPickupCommand = (trigger) -> {
                Command manualPickupCommand = CommandCreator.createPickupCommand.get();
                manualPickupCommand.setName("Manual Pickup");
                trigger.whileTrue(manualPickupCommand);
        };

        public static final Consumer<Trigger> bindManualFenderShootCommand = (trigger) -> {
                Command manualShooterCommand = CommandCreator.createFenderShootCommand.get();
                manualShooterCommand.setName("Manual Fender Shoot");
                trigger.whileTrue(manualShooterCommand);
        };

        public static final Function<DoubleSupplier, Consumer<Trigger>> bindManualShooterTiltCommand = (
                        driveSpeed) -> (trigger) -> {
                                Command manualShooterTiltCommand = RobotContainer.shooterTiltSubsystem.createDriveTiltCommand
                                                .apply(driveSpeed);
                                manualShooterTiltCommand.setName("Manual Shooter Tilt Drive");
                                trigger.whileTrue(manualShooterTiltCommand);
                        };

        public static final Function<DoubleSupplier, Consumer<Trigger>> bindManualElevatorCommand = (
                        driveSpeed) -> (trigger) -> {
                                Command manualShooterTiltCommand = RobotContainer.elevatorSubsystem.createDriveElevatorcommand
                                                .apply(driveSpeed);
                                manualShooterTiltCommand.setName("Manual Elevator Drive");
                                trigger.whileTrue(manualShooterTiltCommand);
                        };

        public static final Consumer<Trigger> bindManualResetSteerEncodersCommand = (trigger) -> {
                trigger.onTrue(RobotContainer.driveSubsystem.createSteerEncodersResetCommand.get());
        };

        public static final Consumer<Trigger> bindManualModuleZeroCommand = (trigger) -> {
                Command zeroModulesCommand = RobotContainer.driveSubsystem.createControlModulesCommand
                                .apply(new SwerveDriveWheelStates(new SwerveModuleState[] {
                                                new SwerveModuleState(),
                                                new SwerveModuleState(),
                                                new SwerveModuleState(),
                                                new SwerveModuleState()
                                }));
                trigger.whileTrue(zeroModulesCommand);
        };


        public static final Function<Pose2d, Function<PathConstraints, Function<Double, Function<Double, Consumer<Trigger>>>>> bindPathFindToPoseCommandToTrigger = (
                        targetPose) -> (constraints) -> (
                                        goalEndVelocityMPS) -> (rotationDelayDistance) -> (trigger) -> {
                                                Supplier<Command> pathFindToPoseCommand = CommandCreator.createPathFindToPoseCommand
                                                                .apply(targetPose)
                                                                .apply(constraints)
                                                                .apply(goalEndVelocityMPS)
                                                                .apply(rotationDelayDistance);

                                                trigger.whileTrue(Commands
                                                                .deferredProxy(pathFindToPoseCommand));
                                                trigger.onFalse(RobotContainer.driveSubsystem.fieldCentricOriginCommand);
                                        };

        public static final Function<Supplier<Optional<Pose2d>>, Function<PathConstraints, Function<Double, Function<Double, Function<Boolean, Consumer<Trigger>>>>>> bindPathFindToSuppliedPoseCommandToTrigger = (
                        targetPose) -> (constraints) -> (
                                        goalEndVelocityMPS) -> (rotationDelayDistance) -> (
                                                        pathFlip) -> (trigger) -> {
                                                                Supplier<Command> setPathFindCommand = CommandCreator.createSetPathFindCommand
                                                                                .apply(targetPose)
                                                                                .apply(constraints)
                                                                                .apply(goalEndVelocityMPS)
                                                                                .apply(rotationDelayDistance)
                                                                                .apply(pathFlip);

                                                                trigger.whileTrue(Commands
                                                                                .deferredProxy(setPathFindCommand));
                                                                trigger.onFalse(RobotContainer.driveSubsystem.fieldCentricOriginCommand);
                                                        };

        public static final Function<Translation2d, Consumer<Trigger>> bindPointToLocationCommandToTrigger = (
                        location) -> (trigger) -> {
                                Command pointToLocationCommand = RobotContainer.driveSubsystem.createPointToLocationCommand
                                                .apply(location);
                                trigger.whileTrue(pointToLocationCommand);
                        };
}
