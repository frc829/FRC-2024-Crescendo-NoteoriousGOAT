package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Value;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Function;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;

public class CommandCreator implements Sendable {

        private static final class Constants {
                private static final Translation2d speakerBlue = new Translation2d(
                                null,
                                null);

                private static final Translation2d speakerRed = new Translation2d(
                                null,
                                null);

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
                        private static final Measure<Distance> bottomOfSpeaker = Meters.of(2.0);
                        private static final Measure<Distance> topOfSpeaker = Meters.of(2.13);
                        private static final Measure<Distance> shooterWheelRadius = Inches.of(2.0);
                        private static final Measure<Distance> shooterTiltX = Inches.of(0.0); // TODO:
                        private static final Measure<Distance> shooterTiltY = Inches.of(0.0); // TODO:
                        private static final Measure<Distance> shooterRadius = Inches.of(0.0); // TODO:
                        private static final Measure<Time> shutOffTime = Seconds.of(2);

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

                        private static final class Ranged {

                        }
                }

                private static final class Amp {

                }
        }

        public static final class ManualCommands {
                public static final Runnable elevatorDrive = () -> RobotContainer.elevatorSubsystem.drive
                                .accept(RobotContainer.operator.leftYValue.getAsDouble());
                public static final Command elevatorDriveCommand = Commands.run(elevatorDrive,
                                RobotContainer.elevatorSubsystem);
                public static final Runnable tiltDrive = () -> RobotContainer.shooterTiltSubsystem.drive
                                .accept(RobotContainer.operator.rightYValue.getAsDouble());
                public static final Command tiltDriveCommand = Commands.run(tiltDrive,
                                RobotContainer.shooterTiltSubsystem);
                public static final Runnable shooterDrive = () -> {
                        RobotContainer.shooterSubsystem.spinTop
                                        .accept(RobotContainer.operator.leftTriggerValue.getAsDouble());
                        RobotContainer.shooterSubsystem.spinBottom
                                        .accept(RobotContainer.operator.rightTriggerValue.getAsDouble());
                };
                public static final Command shooterDriveCommand = Commands.run(shooterDrive,
                                RobotContainer.shooterSubsystem);
        }

        public static final class HoldCommandBuilders {
                public static final Supplier<Command> createElevatorHoldCommand = () -> Commands.run(
                                RobotContainer.elevatorSubsystem.hold,
                                RobotContainer.elevatorSubsystem);

                public static final Supplier<Command> createTiltHoldCommand = () -> Commands.run(
                                RobotContainer.shooterTiltSubsystem.hold,
                                RobotContainer.shooterTiltSubsystem);
        }

        public static final class SetCommandBuilders {
                private static final Measure<Distance> elevatorTolerance = Millimeters.of(10);
                private static final Measure<Angle> tiltTolerance = Degrees.of(0.5);

                public static final Function<Measure<Distance>, Command> createElevatorSetCommand = (position) -> {
                        Runnable setElevator = () -> RobotContainer.elevatorSubsystem.move.accept(position);
                        Command setElevatorCommand = Commands.run(setElevator, RobotContainer.elevatorSubsystem);
                        BooleanSupplier setToHoldTransition = () -> MathUtil.isNear(position.in(Millimeters),
                                        RobotContainer.elevatorSubsystem.position.in(Millimeters),
                                        elevatorTolerance.in(Millimeters));
                        Command command = setElevatorCommand.until(setToHoldTransition)
                                        .andThen(HoldCommandBuilders.createElevatorHoldCommand.get());
                        return command;
                };
                public static final Function<Measure<Angle>, Command> createTiltSetCommand = (angle) -> {
                        Runnable setTilt = () -> RobotContainer.shooterTiltSubsystem.turn.accept(angle);
                        Command setTiltCommand = Commands.run(setTilt, RobotContainer.shooterTiltSubsystem);
                        BooleanSupplier setToHoldTransition = () -> MathUtil.isNear(angle.in(Degrees),
                                        RobotContainer.shooterTiltSubsystem.angle.in(Degrees),
                                        tiltTolerance.in(Degrees));
                        Command command = setTiltCommand.until(setToHoldTransition)
                                        .andThen(HoldCommandBuilders.createTiltHoldCommand.get());
                        return command;
                };
                public static final Function<Double, Function<Double, Command>> createSetShooterCommand = (
                                topSpeed) -> (bottomSpeed) -> {
                                        Runnable setShoot = () -> {
                                                RobotContainer.shooterSubsystem.spinTop.accept(topSpeed);
                                                RobotContainer.shooterSubsystem.spinBottom
                                                                .accept(bottomSpeed);
                                        };
                                        Command command = Commands.run(setShoot, RobotContainer.shooterSubsystem);
                                        return command;
                                };
        }

        public static final class ContinuousCommandBuilders {
                public static final Function<Supplier<Measure<Distance>>, Command> createContinuousElevatorCommand = (
                                position) -> {
                        Runnable setElevator = () -> RobotContainer.elevatorSubsystem.move.accept(position.get());
                        Command command = Commands.run(setElevator,
                                        RobotContainer.elevatorSubsystem);
                        return command;
                };

                public static final Function<Supplier<Measure<Angle>>, Command> createTiltSetCommand = (angle) -> {
                        Runnable setTilt = () -> RobotContainer.shooterTiltSubsystem.turn.accept(angle.get());
                        return Commands.run(setTilt, RobotContainer.shooterTiltSubsystem);
                };
                public static final Function<DoubleSupplier, Function<DoubleSupplier, Command>> createSetShooterCommand = (
                                topSpeed) -> (bottomSpeed) -> {
                                        Runnable setShoot = () -> {
                                                RobotContainer.shooterSubsystem.spinTop.accept(topSpeed.getAsDouble());
                                                RobotContainer.shooterSubsystem.spinBottom
                                                                .accept(bottomSpeed.getAsDouble());
                                        };
                                        return Commands.run(setShoot,
                                                        RobotContainer.shooterSubsystem);
                                };

        }

        public static final class ScoringCommands {
                public static final Supplier<Command> createPickupCommand = () -> {
                        Runnable spinShooters = () -> {
                                RobotContainer.shooterSubsystem.spinTop.accept(Constants.Shoot.Fender.topShooterSpeed);
                                RobotContainer.shooterSubsystem.spinBottom
                                                .accept(Constants.Shoot.Fender.bottomShooterSpeed);
                        };
                        Command spinShooterCommandHasNote = Commands.run(spinShooters, RobotContainer.shooterSubsystem);

                        Runnable spinPickup = () -> {
                                RobotContainer.pickupSubsystem.spinOuter.accept(Constants.Pickup.outerSpeed);
                                RobotContainer.pickupSubsystem.spinInner.accept(Constants.Pickup.innerSpeed);
                                RobotContainer.pickupSubsystem.spinTransport.accept(Constants.Pickup.transportSpeed);
                                RobotContainer.pickupSubsystem.spinSingulator.accept(Constants.Pickup.singulatorSpeed);
                        };

                        Command spinPickupCommand = Commands.run(spinPickup, RobotContainer.pickupSubsystem);
                        Command spinShooterCommandNoHasNote = Commands.run(spinShooters,
                                        RobotContainer.shooterSubsystem);
                        Command spinPickupAndShooter = Commands.parallel(spinPickupCommand,
                                        spinShooterCommandNoHasNote);

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

                public static final Supplier<Command> createFenderShootCommand = () -> {
                        Runnable spinShooters = () -> {
                                RobotContainer.shooterSubsystem.spinTop.accept(Constants.Shoot.Fender.topShooterSpeed);
                                RobotContainer.shooterSubsystem.spinBottom
                                                .accept(Constants.Shoot.Fender.bottomShooterSpeed);
                        };
                        Command spinShootersCommandWaitUntil = Commands.run(spinShooters,
                                        RobotContainer.shooterSubsystem);
                        Command spinShootersCommandFire = Commands.run(spinShooters, RobotContainer.shooterSubsystem);

                        Runnable loadShooter = () -> {
                                RobotContainer.pickupSubsystem.spinTransport
                                                .accept(Constants.Shoot.Fender.transportSpeed);
                                RobotContainer.pickupSubsystem.spinSingulator
                                                .accept(Constants.Shoot.Fender.singulatorSpeed);
                        };
                        Command loadShooterCommand = Commands.run(loadShooter, RobotContainer.pickupSubsystem);

                        Command loadAndShootCommand = Commands.parallel(spinShootersCommandFire, loadShooterCommand);

                        Command command = Commands
                                        .sequence(spinShootersCommandWaitUntil)
                                        .until(() -> MathUtil.isNear(
                                                        RobotContainer.shooterSubsystem.topVelocity.in(Value),
                                                        Constants.Shoot.Fender.topShooterSpeed,
                                                        0.01))
                                        .andThen(loadAndShootCommand)
                                        .withTimeout(Constants.Shoot.shutOffTime.in(Seconds));
                        command.setName("Fender Shoot Command");
                        return command;

                };

                public static final DoubleSupplier robotDistanceToSpeaker = () -> {

                        return 0.0;
                };

                public static Function<Double, Double> getAngleFromDistance = (distance) -> {

                        return 0.0;
                };
        }

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

        public CommandCreator() {
                ManualCommands.elevatorDriveCommand.setName("Manual Elevator");
                ManualCommands.shooterDriveCommand.setName("Manual Shooter");
                ManualCommands.tiltDriveCommand.setName("Manual Tilt");
        }

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
                builder.addDoubleProperty(
                                "Top Shooter Percent",
                                () -> Constants.Pickup.topShooterSpeed,
                                (percent) -> Constants.Pickup.topShooterSpeed = percent);
                builder.addDoubleProperty(
                                "Bottom Shooter Percent",
                                () -> Constants.Pickup.bottomShooterSpeed,
                                (percent) -> Constants.Pickup.bottomShooterSpeed = percent);
        }
}
