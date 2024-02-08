package frc.robot.commands;

import com.controllers.Controller;
import com.pathplanner.lib.path.PathConstraints;

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
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PickupSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterTiltSubsystem;

public class CommandBinder {

        private CommandBinder() {
        }

        public static final Function<PickupSubsystem, Function<ShooterSubsystem, Function<ShooterTiltSubsystem, Consumer<Trigger>>>> bindManualPickupCommand = (
                        pickup) -> (shooter) -> (tilt) -> (trigger) -> {
                                Command manualPickupCommand = CommandCreators.createPickupCommand
                                                .apply(pickup)
                                                .apply(shooter)
                                                .apply(tilt);
                                manualPickupCommand.setName("Manual Pickup");
                                trigger.whileTrue(manualPickupCommand);
                        };

        public static final Function<PickupSubsystem, Function<ShooterSubsystem, Function<ShooterTiltSubsystem, Consumer<Trigger>>>> bindManualFenderShootCommand = (
                        pickup) -> (shooter) -> (tilt) -> (trigger) -> {
                                Command manualShooterCommand = CommandCreators.createFenderShootCommand
                                                .apply(pickup)
                                                .apply(shooter)
                                                .apply(tilt);
                                manualShooterCommand.setName("Manual Fender Shoot");
                                trigger.whileTrue(manualShooterCommand);
                        };

        public static final Function<ShooterTiltSubsystem, Function<DoubleSupplier, Consumer<Trigger>>> bindManualShooterTiltCommand = (
                        shooterTilt) -> (driveSpeed) -> (trigger) -> {
                                Command manualShooterTiltCommand = shooterTilt.createDriveTiltCommand
                                                .apply(driveSpeed);
                                manualShooterTiltCommand.setName("Manual Shooter Tilt Drive");
                                trigger.whileTrue(manualShooterTiltCommand);
                        };

        public static final Function<ElevatorSubsystem, Function<DoubleSupplier, Consumer<Trigger>>> bindManualElevatorCommand = (
                        elevator) -> (driveSpeed) -> (trigger) -> {
                                Command manualShooterTiltCommand = elevator.createDriveElevatorcommand
                                                .apply(driveSpeed);
                                manualShooterTiltCommand.setName("Manual Elevator Drive");
                                trigger.whileTrue(manualShooterTiltCommand);
                        };

        public static final Function<DriveSubsystem, Consumer<Trigger>> bindManualResetSteerEncodersCommand = (
                        drive) -> (trigger) -> {
                                trigger.whileTrue(drive.createSteerEncodersResetCommand.get());
                        };

        public static final Function<DriveSubsystem, Consumer<Trigger>> bindManualModuleZeroCommand = (
                        drive) -> (trigger) -> {
                                Command zeroModulesCommand = drive.createControlModulesCommand
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
                                Command robotCentricCommand = drive.createManualRobotChassisSpeedsCommand
                                                .apply(new Translation2d())
                                                .apply(controller.rightYValue)
                                                .apply(controller.rightXValue)
                                                .apply(controller.fullTriggerValue);
                                robotCentricTrigger.whileTrue(robotCentricCommand);

                        };

        public static final Function<DriveSubsystem, Consumer<Controller>> bindManualFieldCentricDriveCommand = (
                        drive) -> (controller) -> {
                                BooleanSupplier fieldCentricCondition = () -> controller.leftX.getAsBoolean()
                                                || controller.leftY.getAsBoolean();
                                Trigger fieldCentricTrigger = new Trigger(fieldCentricCondition);
                                Command fieldCentricCommand = drive.createManualFieldChassisSpeedsCommand
                                                .apply(new Translation2d())
                                                .apply(controller.leftYValue)
                                                .apply(controller.leftXValue)
                                                .apply(controller.fullTriggerValue);
                                fieldCentricTrigger.whileTrue(fieldCentricCommand);

                        };

        public static final Function<DriveSubsystem, Function<Pose2d, Function<PathConstraints, Function<Double, Function<Double, Consumer<Trigger>>>>>> bindPathFindToPoseCommandToTrigger = (
                        drive) -> (targetPose) -> (constraints) -> (
                                        goalEndVelocityMPS) -> (rotationDelayDistance) -> (trigger) -> {
                                                Command pathFindToPoseCommand = CommandCreators.createPathFindToPoseCommand
                                                                .apply(drive)
                                                                .apply(targetPose)
                                                                .apply(constraints)
                                                                .apply(goalEndVelocityMPS)
                                                                .apply(rotationDelayDistance);

                                                trigger.whileTrue(pathFindToPoseCommand);

                                        };

        public static final Function<DriveSubsystem, Function<Supplier<Optional<Pose2d>>, Function<PathConstraints, Function<Double, Function<Double, Consumer<Trigger>>>>>> bindPathFindToSuppliedPoseCommandToTrigger = (
                        drive) -> (targetPose) -> (constraints) -> (
                                        goalEndVelocityMPS) -> (rotationDelayDistance) -> (trigger) -> {
                                                Command pathFindToPoseCommand = CommandCreators.createPathFindToSuppliedOptPoseCommand
                                                                .apply(drive)
                                                                .apply(targetPose)
                                                                .apply(constraints)
                                                                .apply(goalEndVelocityMPS)
                                                                .apply(rotationDelayDistance);

                                                trigger.whileTrue(pathFindToPoseCommand);
                                                trigger.onFalse(Commands.runOnce(drive.stop, drive));

                                        };
}
