package frc.robot.commandCreators;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Value;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Dimensionless;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import static frc.robot.RobotContainer.*;

public class ScoringCommands {

        private static final class Constants {
                private static final class Amp {
                        private static MutableMeasure<Angle> tiltAngle = MutableMeasure.ofRelativeUnits(58, Degrees);
                        private static MutableMeasure<Distance> elevatorPosition = MutableMeasure.ofBaseUnits(0.30,
                                        Meters);
                        private static double topShooterPercent = -0.6;
                        private static double bottomShooterPercent = -0.6;
                        private static double transportPercent = 0.9;
                        private static double singulatorPercent = -0.9;
                }

                private static final class Fender {
                        private static MutableMeasure<Angle> tiltAngle = MutableMeasure.ofRelativeUnits(58, Degrees);
                        private static Measure<Distance> elevatorPosition = Meters.of(0.0);
                        private static double topShooterPercent = -0.6;
                        private static double bottomShooterPercent = 0.6;
                        private static double transportPercent = 0.9;
                        private static double singulatorPercent = -0.9;
                        private static final double shooterTolerancePercent = 0.10;

                }

                private static final class Ranged {
                        private static final double shooterTolerancePercent = 0.01;
                }

                private static final class SpinUp {
                        private static final double topShooterPercent = Constants.Fender.topShooterPercent * 0.75;
                        private static final double bottomShooterPercent = Constants.Fender.bottomShooterPercent * 0.75;
                }

                private static final class Climb {
                        private static final Measure<Distance> elevatorHeightStart = Meters.of(0.36);
                        private static final Measure<Distance> elevatorHeightEnd = Meters.of(0.10);

                        private static final Measure<Angle> tiltAngle = Degrees.of(0.0);
                }
        }

        public static final Supplier<Command> createClimbPrep = () -> {
                Command command = ResetAndHoldingCommands.setElevatorTiltForever
                                .apply(Constants.Climb.elevatorHeightStart)
                                .apply(Constants.Climb.tiltAngle);
                command.setName("Climb Prep");
                return command;
        };

        public static final Supplier<Command> createClimbEnd = () -> {
                Command command = ResetAndHoldingCommands.setElevatorTiltForever
                                .apply(Constants.Climb.elevatorHeightEnd)
                                .apply(Constants.Climb.tiltAngle);
                command.setName("Climb End");
                return command;
        };

        public static final Supplier<Command> createSpinUp = () -> {
                Runnable spinUp = () -> {
                        topShooterSubsystem.spin.accept(Constants.SpinUp.topShooterPercent);
                        bottomShooterSubsystem.spin.accept(Constants.SpinUp.bottomShooterPercent);
                };
                Command spinUpCommand = Commands.run(spinUp, topShooterSubsystem,
                                bottomShooterSubsystem);
                spinUpCommand.setName("Spin Up");
                return spinUpCommand;
        };

        public static final Supplier<Command> createAmpPosition = () -> {
                Command command = ResetAndHoldingCommands.setElevatorTiltForever
                                .apply(Constants.Amp.elevatorPosition)
                                .apply(Constants.Amp.tiltAngle);

                command.setName("Amp Position");
                return command;
        };

        public static final Supplier<Command> createAmpDrop = () -> {

                Command transportCommand = BasicCommands.Set.Transport.create.apply(Constants.Amp.transportPercent);
                Command singulatorCommand = BasicCommands.Set.Singulator.create.apply(Constants.Amp.singulatorPercent);
                Command topShooterCommand = BasicCommands.Set.TopShooter.create
                                .apply(() -> Constants.Amp.topShooterPercent);
                Command bottomShooterCommand = BasicCommands.Set.BottomShooter.create
                                .apply(() -> Constants.Amp.bottomShooterPercent);

                Command command = Commands
                                .parallel(transportCommand, singulatorCommand,
                                                topShooterCommand, bottomShooterCommand);
                command.setName("Amp Drop");
                return command;
        };

        public static final Supplier<Command> createFender = () -> {
                Command elevatorTiltCommand = ResetAndHoldingCommands.setElevatorTiltUntil
                                .apply(Constants.Fender.elevatorPosition)
                                .apply(Constants.Fender.tiltAngle);

                Runnable speedUpShooters = () -> {
                        topShooterSubsystem.spin.accept(Constants.Fender.topShooterPercent);
                        bottomShooterSubsystem.spin.accept(Constants.Fender.bottomShooterPercent);
                };

                Command speedUpShootersCommand1 = Commands.run(
                                speedUpShooters,
                                topShooterSubsystem,
                                bottomShooterSubsystem);

                Command elevatorTiltShootersCommand = Commands.race(speedUpShootersCommand1, elevatorTiltCommand);

                Command speedUpShootersCommand2 = Commands.run(
                                speedUpShooters,
                                topShooterSubsystem,
                                bottomShooterSubsystem);

                BooleanSupplier shootersAtSpeed = () -> {
                        return MathUtil.isNear(
                                        Constants.Fender.topShooterPercent,
                                        topShooterSubsystem.velocity.in(Value),
                                        Constants.Fender.shooterTolerancePercent);
                };
                Command elevatorHoldCommand = BasicCommands.HoldandStop.createForElevator.get();
                Command elevatorHoldCommand2 = BasicCommands.HoldandStop.createForElevator.get();

                Command tiltHoldCommand = BasicCommands.HoldandStop.createForTilt.get();
                Command tiltHoldCommand2 = BasicCommands.HoldandStop.createForTilt.get();

                Command transportCommand = BasicCommands.Set.Transport.create.apply(Constants.Fender.transportPercent);
                Command singulatorCommand = BasicCommands.Set.Singulator.create
                                .apply(Constants.Fender.singulatorPercent);
                Command topShooterCommand = BasicCommands.Set.TopShooter.create
                                .apply(() -> Constants.Fender.topShooterPercent);
                Command bottomShooterCommand = BasicCommands.Set.BottomShooter.create
                                .apply(() -> Constants.Fender.bottomShooterPercent);

                Command continueUpToSpeed = Commands
                                .parallel(elevatorHoldCommand, tiltHoldCommand, speedUpShootersCommand2)
                                .until(shootersAtSpeed);

                Command shootCommands = Commands
                                .parallel(elevatorHoldCommand2, tiltHoldCommand2, transportCommand, singulatorCommand,
                                                topShooterCommand, bottomShooterCommand);

                Command command = Commands.sequence(elevatorTiltShootersCommand, continueUpToSpeed,
                                shootCommands);
                command.setName("Fender Score");
                return command;
        };

        @SuppressWarnings({ "resource" })
        public static final Supplier<Command> createRanged = () -> {
                MutableMeasure<Angle> tiltAngle = MutableMeasure.zero(Degrees);
                MutableMeasure<Dimensionless> shooterPercentMeasure = MutableMeasure.zero(Value);
                PIDController pidController = new PIDController(5, 0, 0);
                pidController.enableContinuousInput(-180, 180);
                pidController.setTolerance(0.5);

                Runnable setValues = () -> {
                        Pose2d fieldPosition = telemetrySubsystem.poseEstimate.get();
                        Optional<Alliance> alliance = DriverStation.getAlliance();
                        if (alliance.get() == Alliance.Red) {
                                Translation2d targetVector = ResetAndHoldingCommands.Constants.speakerRedVector
                                                .minus(fieldPosition.getTranslation());
                                Rotation2d targetRotation = targetVector.getAngle().rotateBy(Rotation2d.fromDegrees(180));
                                pidController.reset();
                                pidController.setSetpoint(targetRotation.getDegrees());
                                double distanceMeters = targetVector.getNorm();
                                double velocity = 9.8
                                                * (distanceMeters * distanceMeters
                                                                + 4 * ResetAndHoldingCommands.Constants.speakerHeight
                                                                                .in(Meters)
                                                                                * ResetAndHoldingCommands.Constants.speakerHeight
                                                                                                .in(Meters))
                                                / ResetAndHoldingCommands.Constants.speakerHeight.in(Meters);
                                velocity = Math.sqrt(velocity);
                                double shooterOmega = velocity
                                                / ResetAndHoldingCommands.Constants.shooterWheelRadius.in(Meters);
                                shooterOmega /= ResetAndHoldingCommands.Constants.shooterSpeedTransferEfficiency;
                                double shooterPercent = shooterOmega / ResetAndHoldingCommands.Constants.maxShooterSpeed
                                                .in(RadiansPerSecond);
                                shooterPercentMeasure.mut_setMagnitude(shooterPercent);
                                double angleRads = Math
                                                .atan(2 * ResetAndHoldingCommands.Constants.speakerHeight.in(Meters)
                                                                / distanceMeters);
                                double angleDegs = Math.toDegrees(angleRads);
                                tiltAngle.mut_setMagnitude(angleDegs);
                        } else {
                                Translation2d targetVector = ResetAndHoldingCommands.Constants.speakerBlueVector
                                                .minus(fieldPosition.getTranslation());
                                Rotation2d targetRotation = targetVector.getAngle().rotateBy(Rotation2d.fromDegrees(180));
                                pidController.reset();
                                pidController.setSetpoint(targetRotation.getDegrees());
                                double distanceMeters = targetVector.getNorm();
                                double velocity = 9.8
                                                * (distanceMeters * distanceMeters
                                                                + 4 * ResetAndHoldingCommands.Constants.speakerHeight
                                                                                .in(Meters)
                                                                                * ResetAndHoldingCommands.Constants.speakerHeight
                                                                                                .in(Meters))
                                                / ResetAndHoldingCommands.Constants.speakerHeight.in(Meters);
                                velocity = Math.sqrt(velocity);
                                double shooterOmega = velocity
                                                / ResetAndHoldingCommands.Constants.shooterWheelRadius.in(Meters);
                                shooterOmega /= ResetAndHoldingCommands.Constants.shooterSpeedTransferEfficiency;
                                double shooterPercent = shooterOmega / ResetAndHoldingCommands.Constants.maxShooterSpeed
                                                .in(RadiansPerSecond);
                                shooterPercentMeasure.mut_setMagnitude(shooterPercent);
                                double angleRads = Math
                                                .atan(2 * ResetAndHoldingCommands.Constants.speakerHeight.in(Meters)
                                                                / distanceMeters);
                                double angleDegs = Math.toDegrees(angleRads);
                                tiltAngle.mut_setMagnitude(angleDegs);
                        }
                };

                Runnable tilt = () -> {
                        shooterTiltSubsystem.turn.accept(tiltAngle);
                };

                Runnable speedUpShooters = () -> {
                        topShooterSubsystem.spin.accept(shooterPercentMeasure.in(Value));
                        bottomShooterSubsystem.spin.accept(shooterPercentMeasure.in(Value));
                };

                Runnable rotateInPlace = () -> {
                        double currentAngleDegrees = telemetrySubsystem.poseEstimate.get().getRotation()
                                        .getDegrees();
                        currentAngleDegrees %= 360;
                        currentAngleDegrees = currentAngleDegrees >= 180 ? currentAngleDegrees - 360
                                        : currentAngleDegrees;
                        currentAngleDegrees = currentAngleDegrees < -180 ? currentAngleDegrees + 360
                                        : currentAngleDegrees;
                        double degreesPerSecond = pidController.calculate(currentAngleDegrees);
                        double radiansPerSecond = Math.toRadians(degreesPerSecond);
                        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, radiansPerSecond);
                        driveSubsystem.controlRobotChassisSpeeds.apply(new Translation2d())
                                        .accept(chassisSpeeds);
                };

                Command setValuesCommand = Commands.runOnce(setValues);
                Command tiltCommand = Commands.run(tilt, shooterTiltSubsystem);
                Command speedUpShootersCommand = Commands.run(
                                speedUpShooters,
                                topShooterSubsystem,
                                bottomShooterSubsystem);
                Command rotateInPlaceCommand = Commands.run(rotateInPlace, driveSubsystem);

                BooleanSupplier shootersAtSpeed = () -> {
                        return MathUtil.isNear(
                                        shooterPercentMeasure.in(Value),
                                        topShooterSubsystem.velocity.in(Value),
                                        Constants.Ranged.shooterTolerancePercent);
                };

                BooleanSupplier tiltAtPosition = () -> {
                        return MathUtil.isNear(
                                        tiltAngle.in(Degrees),
                                        shooterTiltSubsystem.angle.in(Degrees),
                                        BasicCommands.Set.TiltAngle.tolerance.in(Degrees));
                };

                BooleanSupplier shooterTiltDriveAtSetpoint = () -> {
                        return shootersAtSpeed.getAsBoolean() && tiltAtPosition.getAsBoolean()
                                        && pidController.atSetpoint();
                };

                Command spinUpAndTilt = Commands.parallel(speedUpShootersCommand, tiltCommand, rotateInPlaceCommand)
                                .until(shooterTiltDriveAtSetpoint);

                Command tiltHoldCommand = BasicCommands.HoldandStop.createForTilt.get();
                Command driveStopCommand = Commands.run(driveSubsystem.stop,
                                driveSubsystem);
                Command transportCommand = BasicCommands.Set.Transport.create.apply(Constants.Fender.transportPercent);
                Command singulatorCommand = BasicCommands.Set.Singulator.create
                                .apply(Constants.Fender.singulatorPercent);

                Command shootCommands = Commands
                                .parallel(tiltHoldCommand, driveStopCommand, transportCommand, singulatorCommand);

                Command command = Commands.sequence(setValuesCommand, spinUpAndTilt, shootCommands);
                command.setName("Ranged Score");
                return command;
        };

        public ScoringCommands() {
        }
}
