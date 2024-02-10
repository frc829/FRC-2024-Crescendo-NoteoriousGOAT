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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PickupSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterTiltSubsystem;

public class CommandCreators {

        private static final class Constants {
                private static final class Pickup {
                        private static final DoubleSupplier outerSpeed = () -> 0.5;
                        private static final DoubleSupplier innerSpeed = () -> 0.5;
                        private static final DoubleSupplier transportSpeed = () -> 0.5;
                        private static final DoubleSupplier singulatorSpeed = () -> 0.0;
                        private static final DoubleSupplier topShooterSpeed = () -> 0.5;
                        private static final DoubleSupplier bottomShooterSpeed = () -> 0.5;
                        private static final Supplier<Measure<Angle>> tiltAngle = () -> Degrees.of(60.0);
                        private static final Supplier<Measure<Angle>> angleTolerance = () -> Degrees.of(1.0);
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

        private CommandCreators() {
        }

        public static final Function<PickupSubsystem, Function<ShooterSubsystem, Function<ShooterTiltSubsystem, Command>>> createPickupCommand = (
                        pickup) -> (shooter) -> (tilt) -> {
                                Command pickupCommand = Commands.either(
                                                Commands.parallel(
                                                                tilt.createHoldCommand.get(),
                                                                pickup.createStopCommand.get(),
                                                                shooter.createSpinShootersCommand
                                                                                .apply(Constants.Pickup.topShooterSpeed)
                                                                                .apply(Constants.Pickup.bottomShooterSpeed)),
                                                tilt.createTurnTiltCommand.apply(Constants.Pickup.tiltAngle)
                                                                .until(() -> MathUtil.isNear(
                                                                                Constants.Pickup.tiltAngle.get()
                                                                                                .in(Degrees),
                                                                                tilt.angle.in(Degrees),
                                                                                Constants.Pickup.angleTolerance.get()
                                                                                                .in(Degrees)))
                                                                .andThen(Commands.parallel(
                                                                                tilt.createHoldCommand.get(),
                                                                                pickup.createPickupControlCommand
                                                                                                .apply(Constants.Pickup.outerSpeed)
                                                                                                .apply(Constants.Pickup.innerSpeed)
                                                                                                .apply(Constants.Pickup.transportSpeed)
                                                                                                .apply(Constants.Pickup.singulatorSpeed)
                                                                                                .until(pickup.hasNote)
                                                                                                .andThen(pickup.createStopCommand
                                                                                                                .get()),
                                                                                shooter.createSpinShootersCommand
                                                                                                .apply(Constants.Pickup.topShooterSpeed)
                                                                                                .apply(Constants.Pickup.bottomShooterSpeed))),

                                                pickup.hasNote);
                                pickupCommand.setName("Pickup");
                                return pickupCommand;
                        };

        public static final Function<PickupSubsystem, Function<ShooterSubsystem, Function<ShooterTiltSubsystem, Command>>> createFenderShootCommand = (
                        pickup) -> (shooter) -> (tilt) -> {

                                Command shootCommand = Commands.deadline(
                                                tilt.createTurnTiltCommand.apply(Constants.Shoot.Fender.tiltAngle)
                                                                .until(() -> MathUtil.isNear(
                                                                                Constants.Shoot.Fender.tiltAngle.get()
                                                                                                .in(Degrees),
                                                                                tilt.angle.in(Degrees),
                                                                                Constants.Shoot.Fender.angleTolerance
                                                                                                .get()
                                                                                                .in(Degrees))),
                                                shooter.createSpinShootersCommand
                                                                .apply(Constants.Shoot.Fender.topShooterSpeed)
                                                                .apply(Constants.Shoot.Fender.bottomShooterSpeed))
                                                .andThen(Commands.deadline(
                                                                shooter.createSpinShootersCommand
                                                                                .apply(Constants.Shoot.Fender.topShooterSpeed)
                                                                                .apply(Constants.Shoot.Fender.bottomShooterSpeed)
                                                                                .until(() -> MathUtil.isNear(
                                                                                                Constants.Shoot.Fender.topShooterSpeed
                                                                                                                .getAsDouble(),
                                                                                                shooter.topVelocity.in(
                                                                                                                Value),
                                                                                                Constants.Shoot.Fender.speedTolerance)),
                                                                tilt.createHoldCommand.get()))
                                                .andThen(Commands.waitSeconds(Constants.Shoot.shutOffTime)
                                                                .deadlineWith(
                                                                                tilt.createHoldCommand.get(),
                                                                                pickup.createPickupControlCommand
                                                                                                .apply(Constants.Shoot.Fender.outerSpeed)
                                                                                                .apply(Constants.Shoot.Fender.innerSpeed)
                                                                                                .apply(Constants.Shoot.Fender.transportSpeed)
                                                                                                .apply(Constants.Shoot.Fender.singulatorSpeed),
                                                                                shooter.createSpinShootersCommand
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

        public static final Function<DriveSubsystem, Function<Supplier<Optional<Pose2d>>, Function<PathConstraints, Function<Double, Function<Double, Function<Boolean, Supplier<Command>>>>>>> createSetPathFindCommandSupplier = (
                        drive) -> (targetPoseSupplier) -> (
                                        constraints) -> (goalEndVelocityMPS) -> (
                                                        rotationDelayDistance) -> (pathFlip) -> {
                                                                Supplier<Command> pathFindCommand = () -> {
                                                                        Optional<Pose2d> targetPoseOptional = targetPoseSupplier
                                                                                        .get();
                                                                        if (targetPoseOptional.isPresent()) {
                                                                                Pose2d targetPose = targetPoseOptional
                                                                                                .get();
                                                                                if (pathFlip) {
                                                                                        return AutoBuilder
                                                                                                        .pathfindToPoseFlipped(
                                                                                                                        targetPose,
                                                                                                                        constraints,
                                                                                                                        goalEndVelocityMPS,
                                                                                                                        rotationDelayDistance)
                                                                                                        .handleInterrupt(
                                                                                                                        drive.stop);
                                                                                } else {
                                                                                        return AutoBuilder
                                                                                                        .pathfindToPose(
                                                                                                                        targetPose,
                                                                                                                        constraints,
                                                                                                                        goalEndVelocityMPS,
                                                                                                                        rotationDelayDistance)
                                                                                                        .handleInterrupt(
                                                                                                                        drive.stop);
                                                                                }

                                                                        } else {
                                                                                return Commands.none();
                                                                        }
                                                                };
                                                                return pathFindCommand;

                                                        };

}
