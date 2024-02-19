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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Dimensionless;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;

public class ScoringCommands implements Sendable {

        private static final class Constants {
                private static final class Amp {
                        private static MutableMeasure<Angle> tiltAngle = MutableMeasure.ofRelativeUnits(75, Degrees);
                        private static MutableMeasure<Distance> elevatorPosition = MutableMeasure.ofBaseUnits(0.0,
                                        Meters);
                        private static double topShooterPercent = 0.5;
                        private static double bottomShooterPercent = 0.5;
                        private static double transportPercent = 0.5;
                        private static double singulatorPercent = 0.5;
                }

                private static final class Fender {
                        private static MutableMeasure<Angle> tiltAngle = MutableMeasure.ofRelativeUnits(75, Degrees);
                        private static Measure<Distance> elevatorPosition = Meters.of(0.0);
                        private static double topShooterPercent = 0.5;
                        private static double bottomShooterPercent = 0.5;
                        private static double transportPercent = 0.5;
                        private static double singulatorPercent = 0.5;
                }

                private static final class Ranged {

                        private static double singulatorPercent = 0.0;
                        private static double transportPercent = 0.0;
                        private static final double shooterTolerancePercent = 0.01;
                }
        }

        public static final Supplier<Command> createAmp = () -> {
                Command elevatorTiltCommand = ResetAndHoldingCommands.setElevatorTiltUntil
                                .apply(Constants.Amp.elevatorPosition)
                                .apply(Constants.Amp.tiltAngle);

                Command elevatorHoldCommand = BasicCommands.HoldandStop.createForElevator.get();
                Command tiltHoldCommand = BasicCommands.HoldandStop.createForTilt.get();
                Command transportCommand = BasicCommands.Set.Transport.create.apply(Constants.Amp.transportPercent);
                Command singulatorCommand = BasicCommands.Set.Singulator.create.apply(Constants.Amp.singulatorPercent);
                Command topShooterCommand = BasicCommands.Set.TopShooter.create
                                .apply(() -> Constants.Amp.topShooterPercent);
                Command bottomShooterCommand = BasicCommands.Set.BottomShooter.create
                                .apply(() -> Constants.Amp.bottomShooterPercent);

                Command ampCommands = Commands
                                .parallel(elevatorHoldCommand, tiltHoldCommand, transportCommand, singulatorCommand,
                                                topShooterCommand, bottomShooterCommand);

                Command command = Commands.sequence(elevatorTiltCommand,
                                ampCommands);
                command.setName("Amp Score");
                return command;
        };

        public static final Supplier<Command> createFender = () -> {
                Command elevatorTiltCommand = ResetAndHoldingCommands.setElevatorTiltUntil
                                .apply(Constants.Fender.elevatorPosition)
                                .apply(Constants.Fender.tiltAngle);

                Command elevatorHoldCommand = BasicCommands.HoldandStop.createForElevator.get();
                Command tiltHoldCommand = BasicCommands.HoldandStop.createForTilt.get();
                Command transportCommand = BasicCommands.Set.Transport.create.apply(Constants.Fender.transportPercent);
                Command singulatorCommand = BasicCommands.Set.Singulator.create
                                .apply(Constants.Fender.singulatorPercent);
                Command topShooterCommand = BasicCommands.Set.TopShooter.create
                                .apply(() -> Constants.Fender.topShooterPercent);
                Command bottomShooterCommand = BasicCommands.Set.BottomShooter.create
                                .apply(() -> Constants.Fender.bottomShooterPercent);

                Command shootCommands = Commands
                                .parallel(elevatorHoldCommand, tiltHoldCommand, transportCommand, singulatorCommand,
                                                topShooterCommand, bottomShooterCommand);

                Command command = Commands.sequence(elevatorTiltCommand,
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
                        Pose2d fieldPosition = RobotContainer.telemetrySubsystem.poseEstimate.get();
                        Optional<Alliance> alliance = DriverStation.getAlliance();
                        if (alliance.get() == Alliance.Red) {
                                Translation2d targetVector = ResetAndHoldingCommands.Constants.speakerRedVector
                                                .minus(fieldPosition.getTranslation());
                                pidController.reset();
                                pidController.setSetpoint(targetVector.getAngle().getDegrees());
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
                                pidController.reset();
                                pidController.setSetpoint(targetVector.getAngle().getDegrees());
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
                        RobotContainer.shooterTiltSubsystem.turn.accept(tiltAngle);
                };

                Runnable speedUpShooters = () -> {
                        RobotContainer.topShooterSubsystem.spin.accept(shooterPercentMeasure.in(Value));
                        RobotContainer.bottomShooterSubsystem.spin.accept(shooterPercentMeasure.in(Value));
                };

                Runnable rotateInPlace = () -> {
                        double currentAngleDegrees = RobotContainer.telemetrySubsystem.poseEstimate.get().getRotation()
                                        .getDegrees();
                        currentAngleDegrees %= 360;
                        currentAngleDegrees = currentAngleDegrees >= 180 ? currentAngleDegrees - 360
                                        : currentAngleDegrees;
                        currentAngleDegrees = currentAngleDegrees < -180 ? currentAngleDegrees + 360
                                        : currentAngleDegrees;
                        double degreesPerSecond = pidController.calculate(currentAngleDegrees);
                        double radiansPerSecond = Math.toRadians(degreesPerSecond);
                        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, radiansPerSecond);
                        RobotContainer.driveSubsystem.controlRobotChassisSpeeds.apply(new Translation2d())
                                        .accept(chassisSpeeds);
                };

                Command setValuesCommand = Commands.runOnce(setValues);
                Command tiltCommand = Commands.run(tilt, RobotContainer.shooterTiltSubsystem);
                Command speedUpShootersCommand = Commands.run(
                                speedUpShooters,
                                RobotContainer.topShooterSubsystem,
                                RobotContainer.bottomShooterSubsystem);
                Command rotateInPlaceCommand = Commands.run(rotateInPlace, RobotContainer.driveSubsystem);

                BooleanSupplier shootersAtSpeed = () -> {
                        return MathUtil.isNear(
                                        shooterPercentMeasure.in(Value),
                                        RobotContainer.topShooterSubsystem.velocity.in(Value),
                                        Constants.Ranged.shooterTolerancePercent);
                };

                BooleanSupplier tiltAtPosition = () -> {
                        return MathUtil.isNear(
                                        tiltAngle.in(Degrees),
                                        RobotContainer.shooterTiltSubsystem.angle.in(Degrees),
                                        BasicCommands.Set.TiltAngle.tolerance.in(Degrees));
                };

                BooleanSupplier shooterTiltDriveAtSetpoint = () -> {
                        return shootersAtSpeed.getAsBoolean() && tiltAtPosition.getAsBoolean()
                                        && pidController.atSetpoint();
                };

                Command spinUpAndTilt = Commands.parallel(speedUpShootersCommand, tiltCommand, rotateInPlaceCommand)
                                .until(shooterTiltDriveAtSetpoint);

                Command tiltHoldCommand = BasicCommands.HoldandStop.createForTilt.get();
                Command driveStopCommand = Commands.run(RobotContainer.driveSubsystem.stop, RobotContainer.driveSubsystem);
                Command transportCommand = BasicCommands.Set.Transport.create.apply(Constants.Fender.transportPercent);
                Command singulatorCommand = BasicCommands.Set.Singulator.create
                                .apply(Constants.Fender.singulatorPercent);

                Command shootCommands = Commands
                                .parallel(tiltHoldCommand, driveStopCommand, transportCommand, singulatorCommand);

                Command command = Commands.sequence(setValuesCommand, spinUpAndTilt, shootCommands);
                command.setName("Ranged Score");
                return command;
        };

        static {
                SmartDashboard.putData("Scoring Commands", new ScoringCommands());
        }

        public ScoringCommands() {
        }

        @Override
        public void initSendable(SendableBuilder builder) {
                builder.addDoubleProperty(
                                "Amp Transport Percent",
                                () -> Constants.Amp.transportPercent,
                                (percent) -> Constants.Amp.transportPercent = percent);
                builder.addDoubleProperty(
                                "Amp Singulator Percent",
                                () -> Constants.Amp.singulatorPercent,
                                (percent) -> Constants.Amp.singulatorPercent = percent);
                builder.addDoubleProperty(
                                "Amp Shooter Percent",
                                () -> Constants.Amp.topShooterPercent,
                                (percent) -> {
                                        Constants.Amp.topShooterPercent = percent;
                                        Constants.Amp.bottomShooterPercent = percent;
                                });
                builder.addDoubleProperty(
                                "Amp Elevator Position",
                                () -> Constants.Amp.elevatorPosition.in(Meters),
                                (meters) -> Constants.Amp.elevatorPosition.mut_setMagnitude(meters));
                builder.addDoubleProperty(
                                "Amp Tilt Angle",
                                () -> Constants.Amp.tiltAngle.in(Degrees),
                                (degrees) -> Constants.Amp.tiltAngle.mut_setMagnitude(degrees));

                builder.addDoubleProperty(
                                "Fender Transport Percent",
                                () -> Constants.Fender.transportPercent,
                                (percent) -> Constants.Fender.transportPercent = percent);
                builder.addDoubleProperty(
                                "Fender Singulator Percent",
                                () -> Constants.Fender.singulatorPercent,
                                (percent) -> Constants.Fender.singulatorPercent = percent);
                builder.addDoubleProperty(
                                "Fender Shooter Percent",
                                () -> Constants.Fender.topShooterPercent,
                                (percent) -> {
                                        Constants.Fender.topShooterPercent = percent;
                                        Constants.Fender.bottomShooterPercent = percent;
                                });
                builder.addDoubleProperty(
                                "Fender Tilt Angle",
                                () -> {
                                        var thiny = Constants.Amp.tiltAngle;
                                        return thiny.in(Degrees);
                                },
                                (degrees) -> Constants.Amp.tiltAngle.mut_setMagnitude(degrees));

                builder.addDoubleProperty(
                                "Ranged Transport Percent",
                                () -> Constants.Ranged.transportPercent,
                                (percent) -> Constants.Ranged.transportPercent = percent);
                builder.addDoubleProperty(
                                "Ranged Singulator Percent",
                                () -> Constants.Ranged.singulatorPercent,
                                (percent) -> Constants.Ranged.singulatorPercent = percent);
        }

}
