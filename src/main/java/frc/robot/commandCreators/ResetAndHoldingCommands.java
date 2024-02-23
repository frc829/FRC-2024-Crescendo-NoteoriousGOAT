package frc.robot.commandCreators;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Function;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;

public class ResetAndHoldingCommands implements Sendable {

    public static final class Constants {
        public static final Translation2d speakerBlueVector = new Translation2d(0, 5.544638);
        public static final Translation2d speakerRedVector = new Translation2d(16.542, 5.544638);
        public static final Measure<Distance> speakerHeight = Meters.of(2.065 - Units.inchesToMeters(20 + 4));
        public static final Measure<Distance> shooterWheelRadius = Inches.of(2);
        public static double shooterSpeedTransferEfficiency = 0.80;
        public static final Measure<Velocity<Angle>> maxShooterSpeed = RadiansPerSecond
                .of(DCMotor.getNeoVortex(1).freeSpeedRadPerSec);

    }

    public static final Function<Measure<Distance>, Function<Measure<Angle>, Command>> setElevatorTiltUntil = (
            position) -> (angle) -> {
                Command elevatorCommand = BasicCommands.Set.ElevatorPosition.create.apply(position);
                Command tiltCommand = BasicCommands.Set.TiltAngle.create.apply(angle);

                BooleanSupplier elevatorAtPosition = () -> {
                    return MathUtil.isNear(
                            position.in(Meters),
                            RobotContainer.elevatorSubsystem.position.in(Meters),
                            BasicCommands.Set.ElevatorPosition.tolerance.in(Meters));
                };

                BooleanSupplier tiltAtPosition = () -> {
                    return MathUtil.isNear(
                            angle.in(Degrees),
                            RobotContainer.shooterTiltSubsystem.angle.in(Degrees),
                            BasicCommands.Set.TiltAngle.tolerance.in(Degrees));
                };

                BooleanSupplier elevatorTiltEndCondition = () -> {
                    return elevatorAtPosition.getAsBoolean() && tiltAtPosition.getAsBoolean();
                };

                Command command = Commands.parallel(elevatorCommand, tiltCommand).until(elevatorTiltEndCondition);
                command.setName("Set Elevator Tilt Until");
                return command;
            };

    public static final Function<Measure<Distance>, Function<Measure<Angle>, Command>> setElevatorTiltForever = (
            position) -> (angle) -> {
                Command elevatorCommand = BasicCommands.Set.ElevatorPosition.create.apply(position);
                Command tiltCommand = BasicCommands.Set.TiltAngle.create.apply(angle);

                Command command = Commands.parallel(elevatorCommand, tiltCommand);
                command.setName("Set Elevator Tilt Forever");
                return command;
            };

    public static final Supplier<Command> distanceBasedShooterAdjust = () -> {
        Runnable shooterAdjust = () -> {
            Pose2d fieldPosition = RobotContainer.telemetrySubsystem.poseEstimate.get();
            Optional<Alliance> alliance = DriverStation.getAlliance();

            if (alliance.get() == Alliance.Red) {
                Translation2d targetVector = Constants.speakerRedVector.minus(fieldPosition.getTranslation());
                double distanceMeters = targetVector.getNorm();
                double velocity = 9.8
                        * (distanceMeters * distanceMeters
                                + 4 * Constants.speakerHeight.in(Meters) * Constants.speakerHeight.in(Meters))
                        / Constants.speakerHeight.in(Meters);
                velocity = Math.sqrt(velocity);
                double shooterOmega = velocity / Constants.shooterWheelRadius.in(Meters);
                shooterOmega /= Constants.shooterSpeedTransferEfficiency;
                double shooterPercent = shooterOmega / Constants.maxShooterSpeed.in(RadiansPerSecond);
                RobotContainer.topShooterSubsystem.spin.accept(shooterPercent);
                RobotContainer.bottomShooterSubsystem.spin.accept(shooterPercent);
            } else {
                Translation2d targetVector = Constants.speakerBlueVector.minus(fieldPosition.getTranslation());
                double distanceMeters = targetVector.getNorm();
                double velocity = 9.8
                        * (distanceMeters * distanceMeters
                                + 4 * Constants.speakerHeight.in(Meters) * Constants.speakerHeight.in(Meters))
                        / Constants.speakerHeight.in(Meters);
                velocity = Math.sqrt(velocity);
                double shooterOmega = velocity / Constants.shooterWheelRadius.in(Meters);
                shooterOmega /= Constants.shooterSpeedTransferEfficiency;
                double shooterPercent = shooterOmega / Constants.maxShooterSpeed.in(RadiansPerSecond);
                RobotContainer.topShooterSubsystem.spin.accept(shooterPercent);
                RobotContainer.bottomShooterSubsystem.spin.accept(shooterPercent);
            }
        };
        Command command = Commands.run(shooterAdjust, RobotContainer.topShooterSubsystem,
                RobotContainer.bottomShooterSubsystem);
        command.setName("Distance Based Adjust");
        return command;
    };

    static {
        SmartDashboard.putData("Reset and Holding Commands", new ResetAndHoldingCommands());
    }

    public ResetAndHoldingCommands() {
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty(
            "Shooting Efficiency", 
            () -> Constants.shooterSpeedTransferEfficiency, 
            (efficiency) -> Constants.shooterSpeedTransferEfficiency = efficiency);
    }
}
