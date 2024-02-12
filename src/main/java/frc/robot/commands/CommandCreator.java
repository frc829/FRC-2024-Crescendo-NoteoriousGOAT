package frc.robot.commands;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
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
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;

//pickup, manual and detected
//baby bird
//transfer note to shooter
//shoot the note
//once loaded go to zero degrees, loaded boolean box.  
//amp mode, elevator, tilt, sepate drop mode button, and then reset
//Fender shot, tilt speed, fire
//Ranged shot, just the magic
//climb, elevator up, tilt a little, elevator pull up.  
//trap shot, possible shooter button
//spin up button
//reset pose button
//limelight pose setter

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
                        private static final Measure<Distance> middleOfSpeaker = Meters.of(2.065);
                        private static final Measure<Distance> topOfSpeaker = Meters.of(2.13);
                        private static final Measure<Distance> shooterWheelRadius = Inches.of(2.0);
                        private static final Measure<Distance> shooterTiltX = Centimeters.of(20.0); // TODO:
                        private static final Measure<Distance> shooterTiltY = Inches.of(0.0); // TODO:
                        private static final Measure<Distance> shooterRadius = Centimeters.of(22.0); // TODO:
                        private static final Measure<Time> shutOffTime = Seconds.of(2);
                        private static final double efficiency = 1.0; // TODO:

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

                public static final Supplier<Command> createBabyBirdCommand = () -> {
                        Runnable spinShooters = () -> {
                                RobotContainer.shooterSubsystem.spinTop.accept(-Constants.Shoot.Fender.topShooterSpeed);
                                RobotContainer.shooterSubsystem.spinBottom
                                                .accept(-Constants.Shoot.Fender.bottomShooterSpeed);
                        };
                        Runnable spinPickup = () -> {
                                RobotContainer.pickupSubsystem.spinOuter.accept(0.0);
                                RobotContainer.pickupSubsystem.spinInner.accept(0.0);
                                RobotContainer.pickupSubsystem.spinTransport.accept(0.0);
                                RobotContainer.pickupSubsystem.spinSingulator.accept(-Constants.Pickup.singulatorSpeed);
                        };

                        Command spinPickupCommand = Commands.run(spinPickup, RobotContainer.pickupSubsystem);
                        Command spinShooterCommandNoHasNote = Commands.run(spinShooters,
                                        RobotContainer.shooterSubsystem);

                        BooleanSupplier hasNote = () -> {
                                if (RobotBase.isSimulation()) {
                                        return false;
                                } else {
                                        return RobotContainer.pickupSubsystem.hasNote.getAsBoolean();

                                }
                        };
                        Command command = Commands.parallel(spinPickupCommand,
                                        spinShooterCommandNoHasNote).until(hasNote);
                        command.setName("Baby Bird");
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

                public static final Supplier<Translation2d> robotTranslationToSpeaker = () -> {
                        Pose2d robotPose = new Pose2d(); // TODO: once driveSubsystem is back up
                        var alliance = DriverStation.getAlliance();
                        if (alliance.isPresent()) {
                                if (alliance.get() == Alliance.Blue) {
                                        return Constants.speakerBlue.minus(robotPose.getTranslation());
                                } else {
                                        return Constants.speakerRed.minus(robotPose.getTranslation());
                                }
                        } else {
                                return robotPose.getTranslation();
                        }
                };

                public static final Function<Translation2d, Measure<Angle>> getAngleFromRobotDistanceTranslation = (
                                translation) -> {
                        double robotCenterToSpeakerDistance = translation.getNorm();
                        return Radians.of(Math.atan(2 * Constants.Shoot.middleOfSpeaker.in(Meters))
                                        / robotCenterToSpeakerDistance);
                };

                public static final Function<Translation2d, Measure<Velocity<Distance>>> getVelocityFromRobotDistanceTranslation = (
                                translation) -> {
                        double robotCenterToSpeakerDistance = translation.getNorm();
                        var vSquaredValue = (9.8 * (Math.pow(robotCenterToSpeakerDistance, 2)
                                        + 4 * Math.pow(Constants.Shoot.middleOfSpeaker.in(Meters), 2)))
                                        / Constants.Shoot.middleOfSpeaker.in(Meters);
                        var vValue = Math.sqrt(vSquaredValue);
                        return MetersPerSecond.of(vValue);
                };

                public static final Function<Measure<Velocity<Distance>>, Measure<Velocity<Angle>>> getShooterSpeedFromSpeed = (
                                speed) -> {
                        var omegaRadiansPerSecond = speed.in(MetersPerSecond)
                                        / Constants.Shoot.shooterRadius.in(Meters);
                        return RadiansPerSecond.of(omegaRadiansPerSecond);
                };

                public static final Function<Measure<Velocity<Angle>>, Double> getShooterSpeedPercent = (speed) -> {
                        return speed.in(RadiansPerSecond) / DCMotor.getNeoVortex(1).freeSpeedRadPerSec;
                };

                public static final Supplier<Command> createKeepShootersAtSpeedCommand = () -> {
                        Runnable spinShooters = () -> {
                                var translationFromSpeaker = robotTranslationToSpeaker.get();
                                var velocity = getVelocityFromRobotDistanceTranslation
                                                .andThen(getShooterSpeedFromSpeed)
                                                .andThen(getShooterSpeedPercent)
                                                .apply(translationFromSpeaker);

                                RobotContainer.shooterSubsystem.spinTop
                                                .accept(velocity);
                                RobotContainer.shooterSubsystem.spinBottom
                                                .accept(velocity);
                        };
                        Command spinShootersCommand = Commands.run(
                                        spinShooters,
                                        RobotContainer.shooterSubsystem);
                        Command elevatorDown = Commands.run(
                                        () -> RobotContainer.elevatorSubsystem.move.accept(Meters.of(0)),
                                        RobotContainer.elevatorSubsystem);
                        Command tiltLevel = Commands.run(
                                        () -> RobotContainer.shooterTiltSubsystem.turn.accept(Degrees.of(0)),
                                        RobotContainer.shooterTiltSubsystem);
                        Command command = Commands.parallel(
                                        spinShootersCommand,
                                        elevatorDown,
                                        tiltLevel);
                        command.setName("Keep Shooter At Speed");
                        return command;
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
