package frc.robot.commands;

import com.controllers.Controller;
import com.pathplanner.lib.path.PathConstraints;

import static edu.wpi.first.units.Units.Inches;

import java.util.Optional;
import java.util.function.BooleanSupplier;
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
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

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

        public static final Function<DriveSubsystem, Consumer<Controller>> bindManualRobotCentricDriveCommand = (
                        drive) -> (controller) -> {
                                BooleanSupplier fieldCentricCondition = () -> controller.leftX.getAsBoolean()
                                                || controller.leftY.getAsBoolean();
                                BooleanSupplier robotCentricCondition = () -> !fieldCentricCondition.getAsBoolean()
                                                &&
                                                (controller.rightX.getAsBoolean() ||
                                                                controller.rightY.getAsBoolean() ||
                                                                controller.fullTrigger.getAsBoolean());
                                Trigger robotCentricTrigger = new Trigger(robotCentricCondition);
                                Supplier<Translation2d> corSupplier = () -> {
                                        if (controller.rightBumper.getAsBoolean()) {
                                                return new Translation2d(Inches.of(13), Inches.of(-13));
                                        } else if (controller.leftBumper.getAsBoolean()) {
                                                return new Translation2d(Inches.of(13), Inches.of(13));
                                        } else {
                                                return new Translation2d();
                                        }
                                };
                                Command robotCentricCommand = drive.createManualRobotChassisSpeedsCommand
                                                .apply(corSupplier)
                                                .apply(controller.rightYValue)
                                                .apply(controller.rightXValue)
                                                .apply(controller.fullTriggerValue);
                                robotCentricTrigger.whileTrue(robotCentricCommand);

                        };

        public static final Function<DriveSubsystem, Function<Controller, Command>> bindManualFieldCentricDriveCommand = (
                        drive) -> (controller) -> {
                                BooleanSupplier fieldCentricCondition = () -> controller.leftX.getAsBoolean()
                                                || controller.leftY.getAsBoolean();
                                Trigger fieldCentricTrigger = new Trigger(fieldCentricCondition);
                                Supplier<Translation2d> corSupplier = () -> {
                                        if (controller.rightBumper.getAsBoolean()) {
                                                return new Translation2d(Inches.of(13), Inches.of(-13));
                                        } else if (controller.leftBumper.getAsBoolean()) {
                                                return new Translation2d(Inches.of(13), Inches.of(13));
                                        } else {
                                                return new Translation2d();
                                        }
                                };
                                Command fieldCentricCommand = drive.createManualFieldChassisSpeedsCommand
                                                .apply(corSupplier)
                                                .apply(controller.leftYValue)
                                                .apply(controller.leftXValue)
                                                .apply(controller.fullTriggerValue);
                                fieldCentricTrigger.whileTrue(fieldCentricCommand);
                                return fieldCentricCommand;

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
                                                trigger.onFalse(RobotContainer.fieldCentricCommand);
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
                                                                trigger.onFalse(RobotContainer.fieldCentricCommand);
                                                        };

        public static final Function<Translation2d, Consumer<Trigger>> bindPointToLocationCommandToTrigger = (
                        location) -> (trigger) -> {
                                Command pointToLocationCommand = RobotContainer.driveSubsystem.createPointToLocationCommand
                                                .apply(location);
                                trigger.whileTrue(pointToLocationCommand);
                        };
}
