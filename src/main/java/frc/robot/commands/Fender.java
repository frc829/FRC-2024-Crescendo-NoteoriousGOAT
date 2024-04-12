package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static frc.robot.RobotContainer.bottomShooterSubsystem;
import static frc.robot.RobotContainer.singulatorSubsystem;
import static frc.robot.RobotContainer.topShooterSubsystem;
import static frc.robot.RobotContainer.transportSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public abstract class Fender {
        private static final class Constants {
                private static Measure<Angle> tiltAngle = Degrees.of(57); // 57 dege
                private static Measure<Distance> elevatorPosition = Meters.of(0.0);
                private static double topShooterPercent = -0.7;
                private static double bottomShooterPercent = 0.7;
                private static double transportPercent = 0.9;
                private static double singulatorPercent = -0.9;
                private static final double shooterTolerancePercent = 0.10;
                private static final double endOfShootDelay = 0.2;

        }

        private static final BooleanSupplier shootersAtSpeed = () -> {
                boolean topCondition = topShooterSubsystem.atSpeed(Constants.topShooterPercent,
                                Constants.shooterTolerancePercent);
                boolean bottomCondition = bottomShooterSubsystem.atSpeed(Constants.bottomShooterPercent,
                                Constants.shooterTolerancePercent);
                boolean condition = topCondition && bottomCondition;
                SmartDashboard.putBoolean("ShooterAt Speed", condition);
                return condition;
        };

        private static final BooleanSupplier elevatorAndTiltAtPositionCondition = () -> {
                return BasicCommands.Elevator.createAtPositionCondition.apply(Constants.elevatorPosition)
                                .getAsBoolean() &&
                                BasicCommands.Tilt.createAtAngleCondition.apply(Constants.tiltAngle)
                                                .getAsBoolean();

        };

        private static final Supplier<Command> createAimCommand = () -> {
                Command command = Commands.parallel(
                                BasicCommands.Elevator.createSetAndHoldElevatorPositionCommand
                                                .apply(Constants.elevatorPosition),
                                BasicCommands.Tilt.createSetAndHoldTiltAngleCommand.apply(Constants.tiltAngle),
                                topShooterSubsystem.createSetVelocityRatioCommand(() -> Constants.topShooterPercent),
                                bottomShooterSubsystem
                                                .createSetVelocityRatioCommand(() -> Constants.bottomShooterPercent))
                                .until(elevatorAndTiltAtPositionCondition);
                command.setName("Fender Aim");
                return command;
        };

        private static final Supplier<Command> createSpinUp = () -> {
                Command command = Commands.parallel(
                                BasicCommands.Elevator.createSetAndHoldElevatorPositionCommand
                                                .apply(Constants.elevatorPosition),
                                BasicCommands.Tilt.createSetAndHoldTiltAngleCommand.apply(Constants.tiltAngle),
                                topShooterSubsystem.createSetVelocityRatioCommand(() -> Constants.topShooterPercent),
                                bottomShooterSubsystem
                                                .createSetVelocityRatioCommand(() -> Constants.bottomShooterPercent))
                                .until(shootersAtSpeed);
                command.setName("Fender Spin Up");
                return command;
        };

        private static final Supplier<Command> createShootCommand = () -> {
                Command command = Commands.parallel(
                                BasicCommands.Elevator.createSetAndHoldElevatorPositionCommand
                                                .apply(Constants.elevatorPosition),
                                BasicCommands.Tilt.createSetAndHoldTiltAngleCommand.apply(Constants.tiltAngle),
                                topShooterSubsystem.createSetVelocityRatioCommand(() -> Constants.topShooterPercent),
                                bottomShooterSubsystem
                                                .createSetVelocityRatioCommand(() -> Constants.bottomShooterPercent),
                                singulatorSubsystem.createSetVelocityRatioCommand(() -> Constants.singulatorPercent),
                                transportSubsystem.createSetVelocityRatioCommand(() -> Constants.transportPercent));
                command.setName("Fender Shoot");
                return command;
        };

        public static final Supplier<Command> create = () -> {
                return Commands.sequence(createAimCommand.get(), createSpinUp.get(), createShootCommand.get());
        };

        public static final Supplier<Command> createWithDelay = () -> {
                return Commands.sequence(
                                createAimCommand.get(),
                                createSpinUp.get(),
                                createShootCommand.get()
                                                .raceWith(Commands.waitSeconds(Constants.endOfShootDelay)),
                                                singulatorSubsystem.createStopCommand());
        };
}