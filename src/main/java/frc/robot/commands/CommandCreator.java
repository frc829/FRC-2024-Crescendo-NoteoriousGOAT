package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Function;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.RobotContainer;

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
                                private static final double outerSpeed = 0.0;
                                private static final double innerSpeed = 0.0;
                                private static final double transportSpeed = 0.5;
                                private static final double singulatorSpeed = 0.5;
                                private static final double topShooterSpeed = 0.5;
                                private static final double bottomShooterSpeed = 0.5;
                                private static final double speedTolerance = 0.05;
                                private static final Supplier<Measure<Angle>> tiltAngle = () -> Degrees.of(80.0);
                                private static final Supplier<Measure<Angle>> angleTolerance = () -> Degrees.of(1.0);

                        }
                }
        }

        static {
                new CommandCreator();
        }

        private CommandCreator() {
                SmartDashboard.putData("CommandCreator", this);
        }

        public static final Supplier<Command> createPickupCommand = () -> {
                Runnable spinShooters = () -> {
                        RobotContainer.shooterSubsystem.spinTop.accept(Constants.Shoot.Fender.topShooterSpeed);
                        RobotContainer.shooterSubsystem.spinBottom.accept(Constants.Shoot.Fender.bottomShooterSpeed);
                };
                Command spinShooterCommandHasNote = Commands.run(spinShooters, RobotContainer.shooterSubsystem);

                Runnable spinPickup = () -> {
                        RobotContainer.pickupSubsystem.spinOuter.accept(Constants.Pickup.outerSpeed);
                        RobotContainer.pickupSubsystem.spinInner.accept(Constants.Pickup.innerSpeed);
                        RobotContainer.pickupSubsystem.spinTransport.accept(Constants.Pickup.transportSpeed);
                        RobotContainer.pickupSubsystem.spinSingulator.accept(Constants.Pickup.singulatorSpeed);
                };

                Command spinPickupCommand = Commands.run(spinPickup, RobotContainer.pickupSubsystem);
                Command spinShooterCommandNoHasNote = Commands.run(spinShooters, RobotContainer.shooterSubsystem);
                Command spinPickupAndShooter = Commands.parallel(spinPickupCommand, spinShooterCommandNoHasNote);

                BooleanSupplier hasNote = () -> {
                        if (RobotBase.isSimulation()) {
                                return false;
                        } else {
                                return RobotContainer.pickupSubsystem.hasNote.getAsBoolean();

                        }
                };

                Command command = Commands.either(
                                spinShooterCommandHasNote,
                                spinPickupAndShooter,
                                hasNote);
                command.setName("Pickup Command");
                return command;
        };

        // public static final Supplier<Command> createFenderShootCommand = () -> {
        // Command shootCommand = Commands.deadline(
        // RobotContainer.shooterTiltSubsystem.createTurnTiltCommand
        // .apply(Constants.Shoot.Fender.tiltAngle)
        // .until(() -> MathUtil.isNear(
        // Constants.Shoot.Fender.tiltAngle.get()
        // .in(Degrees),
        // RobotContainer.shooterTiltSubsystem.angle.in(Degrees),
        // Constants.Shoot.Fender.angleTolerance
        // .get()
        // .in(Degrees))),
        // RobotContainer.shooterSubsystem.createSpinShootersCommand
        // .apply(Constants.Shoot.Fender.topShooterSpeed)
        // .apply(Constants.Shoot.Fender.bottomShooterSpeed))
        // .andThen(Commands.deadline(
        // RobotContainer.shooterSubsystem.createSpinShootersCommand
        // .apply(Constants.Shoot.Fender.topShooterSpeed)
        // .apply(Constants.Shoot.Fender.bottomShooterSpeed)
        // .until(() -> MathUtil.isNear(
        // Constants.Shoot.Fender.topShooterSpeed
        // .getAsDouble(),
        // RobotContainer.shooterSubsystem.topVelocity
        // .in(
        // Value),
        // Constants.Shoot.Fender.speedTolerance)),
        // RobotContainer.shooterTiltSubsystem.createHoldCommand.get()))
        // .andThen(Commands.waitSeconds(Constants.Shoot.shutOffTime)
        // .deadlineWith(
        // RobotContainer.shooterTiltSubsystem.createHoldCommand
        // .get(),
        // RobotContainer.pickupSubsystem.createPickupControlCommand
        // .apply(Constants.Shoot.Fender.outerSpeed)
        // .apply(Constants.Shoot.Fender.innerSpeed)
        // .apply(Constants.Shoot.Fender.transportSpeed)
        // .apply(Constants.Shoot.Fender.singulatorSpeed),
        // RobotContainer.shooterSubsystem.createSpinShootersCommand
        // .apply(Constants.Shoot.Fender.topShooterSpeed)
        // .apply(Constants.Shoot.Fender.bottomShooterSpeed)));

        // shootCommand.setName("Fender Shoot");
        // return shootCommand;
        // };

        public static final Function<Pose2d, Function<PathConstraints, Function<Double, Function<Double, Supplier<Command>>>>> createPathFindToPoseCommand = (
                        targetPose) -> (constraints) -> (goalEndVelocityMPS) -> (rotationDelayDistance) -> {
                                Command pathFindToPoseCommand = AutoBuilder.pathfindToPoseFlipped(
                                                targetPose,
                                                constraints,
                                                goalEndVelocityMPS,
                                                rotationDelayDistance);
                                pathFindToPoseCommand.setName("PathFindToFixedPose");
                                return () -> pathFindToPoseCommand;
                        };

        public static final Command[] pathFindToSuppliedOptPoseCommand = new Command[] { Commands.none() };

        // public static final Function<Supplier<Optional<Pose2d>>,
        // Function<PathConstraints, Function<Double, Function<Double, Function<Boolean,
        // Supplier<Command>>>>>> createSetPathFindCommand = (
        // targetPoseSupplier) -> (
        // constraints) -> (goalEndVelocityMPS) -> (
        // rotationDelayDistance) -> (pathFlip) -> {
        // return () -> {
        // Optional<Pose2d> targetPoseOptional = targetPoseSupplier
        // .get();
        // if (targetPoseOptional.isPresent()) {
        // Pose2d targetPose = targetPoseOptional
        // .get();
        // if (pathFlip) {
        // return AutoBuilder
        // .pathfindToPoseFlipped(
        // targetPose,
        // constraints,
        // goalEndVelocityMPS,
        // rotationDelayDistance)
        // .handleInterrupt(
        // RobotContainer.driveSubsystem.stop);
        // } else {
        // return AutoBuilder
        // .pathfindToPose(
        // targetPose,
        // constraints,
        // goalEndVelocityMPS,
        // rotationDelayDistance)
        // .handleInterrupt(
        // RobotContainer.driveSubsystem.stop);
        // }

        // } else {
        // return Commands.none();
        // }
        // };
        // };

        @Override
        public void initSendable(SendableBuilder builder) {
                builder.addDoubleProperty(
                                "Outer Percent",
                                () -> Constants.Pickup.outerSpeed,
                                (percent) -> Constants.Pickup.outerSpeed = percent / 100);
                builder.addDoubleProperty(
                                "Inner Percent",
                                () -> Constants.Pickup.innerSpeed,
                                (percent) -> Constants.Pickup.innerSpeed = percent / 100);
                builder.addDoubleProperty(
                                "Transport Percent",
                                () -> Constants.Pickup.transportSpeed,
                                (percent) -> Constants.Pickup.transportSpeed = percent / 100);
                builder.addDoubleProperty(
                                "Singulator Percent",
                                () -> Constants.Pickup.singulatorSpeed,
                                (percent) -> Constants.Pickup.singulatorSpeed = percent / 100);
                builder.addDoubleProperty(
                                "Top Shooter Percent",
                                () -> Constants.Pickup.topShooterSpeed,
                                (percent) -> Constants.Pickup.topShooterSpeed = percent / 100);
                builder.addDoubleProperty(
                                "Bottom Shooter Percent",
                                () -> Constants.Pickup.bottomShooterSpeed,
                                (percent) -> Constants.Pickup.bottomShooterSpeed = percent / 100);
        }
}
