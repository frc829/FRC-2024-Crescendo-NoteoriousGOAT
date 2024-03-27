package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static frc.robot.RobotContainer.*;
import java.util.function.Supplier;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public abstract class Amp {
        private static final class Constants {
                private static Measure<Angle> tiltAngle = Degrees.of(48);
                private static Measure<Distance> elevatorPosition = Meters.of(0.325);
                private static double topShooterPercent = -0.9;
                private static double bottomShooterPercent = -0.8;
                private static double transportPercent = 0.9;
                private static double singulatorPercent = -0.7;
        }

        public static final Supplier<Command> createAmpPosition = () -> {
                Command command = Commands.parallel(
                        topShooterSubsystem.createSetSpeedCommand(() -> Constants.topShooterPercent),
                        bottomShooterSubsystem.createSetSpeedCommand(() -> Constants.bottomShooterPercent),
                                BasicCommands.Elevator.createSetAndHoldElevatorPositionCommand
                                                .apply(Constants.elevatorPosition),
                                BasicCommands.Tilt.createSetAndHoldTiltAngleCommand.apply(Constants.tiltAngle));
                command.setName("Amp Position");
                return command;
        };

        public static final Supplier<Command> createAmpDrop = () -> {
                Command command = Commands.parallel(
                        topShooterSubsystem.createSetSpeedCommand(() -> Constants.topShooterPercent),
                        bottomShooterSubsystem.createSetSpeedCommand(() -> Constants.bottomShooterPercent),

                                BasicCommands.Elevator.createSetAndHoldElevatorPositionCommand
                                                .apply(Constants.elevatorPosition),
                                BasicCommands.Tilt.createSetAndHoldTiltAngleCommand.apply(Constants.tiltAngle),
                                singulatorSubsystem.createSetSpeedCommand(() -> Constants.singulatorPercent),
                                BasicCommands.Transport.createSpinCommand.apply(Constants.transportPercent));

                command.setName("Amp Drop");
                return command;
        };
}