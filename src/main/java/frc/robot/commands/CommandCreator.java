package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Value;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Function;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

public class CommandCreator implements Sendable {

        private static final class Constants {
                private static final class Pickup {
                        private static double outerSpeed = 0.5;
                        private static double innerSpeed = 0.5;
                        private static double transportSpeed = 0.5;
                        private static double singulatorSpeed = 0.0;
                        private static double topShooterSpeed = 0.5;
                        private static double bottomShooterSpeed = 0.5;
                        private static Supplier<Measure<Angle>> tiltAngle = () -> Degrees.of(60.0);
                        private static Supplier<Measure<Angle>> angleTolerance = () -> Degrees.of(1.0);
                }

                private static final class Shoot {
                        private static final double bottomOfSpeakerMeters = 2.0;
                        private static final double topOfSpeakerMeters = 2.13;
                        private static final double shooterWheelRadiusInches = 2.0;
                        private static final double shooterTiltX = 0.0; // TODO:
                        private static final double shooterTiltY = 0.0; // TODO:
                        private static final double shooterRadius = 0.0; // TODO:
                        private static final double shutOffTime = 5;

                        private static final class Fender {
                                private static final DoubleSupplier outerSpeed = () -> 0.0;
                                private static final DoubleSupplier innerSpeed = () -> 0.0;
                                private static final DoubleSupplier transportSpeed = () -> 0.5;
                                private static final DoubleSupplier singulatorSpeed = () -> 0.5;
                                private static final DoubleSupplier topShooterSpeed = () -> 0.5;
                                private static final DoubleSupplier bottomShooterSpeed = () -> 0.5;
                                private static final double speedTolerance = 0.05;
                                private static final Supplier<Measure<Angle>> tiltAngle = () -> Degrees.of(80.0);
                                private static final Supplier<Measure<Angle>> angleTolerance = () -> Degrees.of(1.0);

                        }
                }
        }

        private CommandCreator() {
                SmartDashboard.putData("CommandCreator", this);
        }

        public static final Supplier<Command> createPickupCommand = () -> {
                Command pickupCommand = Commands.either(
                                Commands.parallel(
                                                RobotContainer.shooterTiltSubsystem.createHoldCommand.get(),
                                                RobotContainer.pickupSubsystem.createStopCommand.get(),
                                                RobotContainer.shooterSubsystem.createSpinShootersCommand
                                                                .apply(() -> Constants.Pickup.topShooterSpeed)
                                                                .apply(() -> Constants.Pickup.bottomShooterSpeed)),
                                RobotContainer.shooterTiltSubsystem.createTurnTiltCommand
                                                .apply(Constants.Pickup.tiltAngle)
                                                .until(() -> MathUtil.isNear(
                                                                Constants.Pickup.tiltAngle.get()
                                                                                .in(Degrees),
                                                                RobotContainer.shooterTiltSubsystem.angle.in(Degrees),
                                                                Constants.Pickup.angleTolerance.get()
                                                                                .in(Degrees)))
                                                .andThen(Commands.parallel(
                                                                RobotContainer.shooterTiltSubsystem.createHoldCommand
                                                                                .get(),
                                                                RobotContainer.pickupSubsystem.createPickupControlCommand
                                                                                .apply(() -> Constants.Pickup.outerSpeed)
                                                                                .apply(() -> Constants.Pickup.innerSpeed)
                                                                                .apply(() -> Constants.Pickup.transportSpeed)
                                                                                .apply(() -> Constants.Pickup.singulatorSpeed)
                                                                                .until(RobotContainer.pickupSubsystem.hasNote)
                                                                                .andThen(RobotContainer.pickupSubsystem.createStopCommand
                                                                                                .get()),
                                                                RobotContainer.shooterSubsystem.createSpinShootersCommand
                                                                                .apply(() -> Constants.Pickup.topShooterSpeed)
                                                                                .apply(() -> Constants.Pickup.bottomShooterSpeed))),

                                RobotContainer.pickupSubsystem.hasNote);
                pickupCommand.setName("Pickup");
                return pickupCommand;
        };

        public static final Supplier<Command> createFenderShootCommand = () -> {
                Command shootCommand = Commands.deadline(
                                RobotContainer.shooterTiltSubsystem.createTurnTiltCommand
                                                .apply(Constants.Shoot.Fender.tiltAngle)
                                                .until(() -> MathUtil.isNear(
                                                                Constants.Shoot.Fender.tiltAngle.get()
                                                                                .in(Degrees),
                                                                RobotContainer.shooterTiltSubsystem.angle.in(Degrees),
                                                                Constants.Shoot.Fender.angleTolerance
                                                                                .get()
                                                                                .in(Degrees))),
                                RobotContainer.shooterSubsystem.createSpinShootersCommand
                                                .apply(Constants.Shoot.Fender.topShooterSpeed)
                                                .apply(Constants.Shoot.Fender.bottomShooterSpeed))
                                .andThen(Commands.deadline(
                                                RobotContainer.shooterSubsystem.createSpinShootersCommand
                                                                .apply(Constants.Shoot.Fender.topShooterSpeed)
                                                                .apply(Constants.Shoot.Fender.bottomShooterSpeed)
                                                                .until(() -> MathUtil.isNear(
                                                                                Constants.Shoot.Fender.topShooterSpeed
                                                                                                .getAsDouble(),
                                                                                RobotContainer.shooterSubsystem.topVelocity
                                                                                                .in(
                                                                                                                Value),
                                                                                Constants.Shoot.Fender.speedTolerance)),
                                                RobotContainer.shooterTiltSubsystem.createHoldCommand.get()))
                                .andThen(Commands.waitSeconds(Constants.Shoot.shutOffTime)
                                                .deadlineWith(
                                                                RobotContainer.shooterTiltSubsystem.createHoldCommand
                                                                                .get(),
                                                                RobotContainer.pickupSubsystem.createPickupControlCommand
                                                                                .apply(Constants.Shoot.Fender.outerSpeed)
                                                                                .apply(Constants.Shoot.Fender.innerSpeed)
                                                                                .apply(Constants.Shoot.Fender.transportSpeed)
                                                                                .apply(Constants.Shoot.Fender.singulatorSpeed),
                                                                RobotContainer.shooterSubsystem.createSpinShootersCommand
                                                                                .apply(Constants.Shoot.Fender.topShooterSpeed)
                                                                                .apply(Constants.Shoot.Fender.bottomShooterSpeed)));

                shootCommand.setName("Fender Shoot");
                return shootCommand;
        };

        public static final Function<DriveSubsystem, Function<Pose2d, Function<PathConstraints, Function<Double, Function<Double, Command>>>>> createPathFindToPoseCommand = (
                        drive) -> (targetPose) -> (constraints) -> (goalEndVelocityMPS) -> (rotationDelayDistance) -> {
                                Command pathFindToPoseCommand = AutoBuilder.pathfindToPoseFlipped(
                                                targetPose,
                                                constraints,
                                                goalEndVelocityMPS,
                                                rotationDelayDistance);
                                pathFindToPoseCommand.setName("PathFindToFixedPose");
                                return pathFindToPoseCommand;

                        };

        public static final Command[] pathFindToSuppliedOptPoseCommand = new Command[] { Commands.none() };

        public static final Function<DriveSubsystem, Function<Supplier<Optional<Pose2d>>, Function<PathConstraints, Function<Double, Function<Double, Function<Boolean, Command>>>>>> createSetPathFindCommand = (
                        drive) -> (targetPoseSupplier) -> (
                                        constraints) -> (goalEndVelocityMPS) -> (
                                                        rotationDelayDistance) -> (pathFlip) -> {
                                                                Runnable setPathFind = () -> {
                                                                        Optional<Pose2d> targetPoseOptional = targetPoseSupplier
                                                                                        .get();
                                                                        if (targetPoseOptional.isPresent()) {
                                                                                Pose2d targetPose = targetPoseOptional
                                                                                                .get();
                                                                                if (pathFlip) {
                                                                                        pathFindToSuppliedOptPoseCommand[0] = AutoBuilder
                                                                                                        .pathfindToPoseFlipped(
                                                                                                                        targetPose,
                                                                                                                        constraints,
                                                                                                                        goalEndVelocityMPS,
                                                                                                                        rotationDelayDistance)
                                                                                                        .handleInterrupt(
                                                                                                                        drive.stop);
                                                                                } else {
                                                                                        pathFindToSuppliedOptPoseCommand[0] = AutoBuilder
                                                                                                        .pathfindToPose(
                                                                                                                        targetPose,
                                                                                                                        constraints,
                                                                                                                        goalEndVelocityMPS,
                                                                                                                        rotationDelayDistance)
                                                                                                        .handleInterrupt(
                                                                                                                        drive.stop);
                                                                                }

                                                                        } else {
                                                                                pathFindToSuppliedOptPoseCommand[0] = Commands
                                                                                                .none();
                                                                        }
                                                                        pathFindToSuppliedOptPoseCommand[0]
                                                                                        .setName("PathFindToSuppliedPose");
                                                                        pathFindToSuppliedOptPoseCommand[0].schedule();
                                                                };
                                                                Command setPathFindCommand = Commands
                                                                                .runOnce(setPathFind);
                                                                return setPathFindCommand;
                                                        };

        @Override
        public void initSendable(SendableBuilder builder) {
                builder.addDoubleProperty(
                                "Outer Percent",
                                () -> Constants.Pickup.outerSpeed,
                                (percent) -> Constants.Pickup.outerSpeed = percent);
                builder.addDoubleProperty(
                                "Inner Percent",
                                () -> Constants.Pickup.innerSpeed,
                                (percent) -> Constants.Pickup.innerSpeed = percent);
                builder.addDoubleProperty(
                                "Transport Percent",
                                () -> Constants.Pickup.transportSpeed,
                                (percent) -> Constants.Pickup.transportSpeed = percent);
                builder.addDoubleProperty(
                                "Singulator Percent",
                                () -> Constants.Pickup.singulatorSpeed,
                                (percent) -> Constants.Pickup.singulatorSpeed = percent);
        }
}
