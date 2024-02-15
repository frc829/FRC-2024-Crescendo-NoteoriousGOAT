package frc.robot.commandCreators;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Millimeters;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.Supplier;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;

public class BasicCommands {

    public static final class HoldandStop {
        public static final Supplier<Command> createForElevator = () -> Commands.run(
                RobotContainer.elevatorSubsystem.hold,
                RobotContainer.elevatorSubsystem);

        public static final Supplier<Command> createForTilt = () -> Commands.run(
                RobotContainer.shooterTiltSubsystem.hold,
                RobotContainer.shooterTiltSubsystem);
    }

    public static final class Set {
        public static final class ElevatorPosition {
            public static final Measure<Distance> tolerance = Millimeters.of(5);
            public static final Function<Measure<Distance>, Command> create = (position) -> {
                Command command = Commands
                        .run(() -> RobotContainer.elevatorSubsystem.move.accept(position),
                                RobotContainer.elevatorSubsystem);
                String name = String.format("Move Elevator to %s m", position.in(Meters));
                command.setName(name);
                return command;
            };
        }

        public static final class ElevatorDrive {
            public static final Function<DoubleSupplier, Command> create = (speed) -> {
                Command command = Commands
                        .run(() -> RobotContainer.elevatorSubsystem.drive.accept(speed.getAsDouble()),
                                RobotContainer.elevatorSubsystem);
                String name = String.format("Drive Elevator");
                command.setName(name);
                return command;
            };
        }

        public static final class TiltAngle {
            public static final Measure<Angle> tolerance = Degrees.of(0.5);
            public static final Function<Measure<Angle>, Command> create = (angle) -> {
                Command command = Commands
                        .run(() -> RobotContainer.shooterTiltSubsystem.turn.accept(angle),
                                RobotContainer.shooterTiltSubsystem);
                String name = String.format("Move Tilt to %s degrees", angle.in(Degrees));
                command.setName(name);
                return command;
            };
        }

        public static final class TiltDrive {
            public static final Function<DoubleSupplier, Command> create = (speed) -> {
                Command command = Commands
                        .run(() -> RobotContainer.shooterTiltSubsystem.drive.accept(speed.getAsDouble()),
                                RobotContainer.shooterTiltSubsystem);
                String name = String.format("Drive Tilt");
                command.setName(name);
                return command;
            };
        }

        public static final class TopShooter {
            public static final Function<DoubleSupplier, Command> create = (speed) -> {
                Command command = Commands
                        .run(() -> RobotContainer.topShooterSubsystem.spin.accept(speed.getAsDouble()),
                                RobotContainer.topShooterSubsystem);
                String name = String.format("Accelerate Top Shooter");
                command.setName(name);
                return command;
            };
        }

        public static final class BottomShooter {
            public static final Function<DoubleSupplier, Command> create = (speed) -> {
                Command command = Commands
                        .run(() -> RobotContainer.bottomShooterSubsystem.spin.accept(speed.getAsDouble()),
                                RobotContainer.bottomShooterSubsystem);
                String name = String.format("Accelerate Bottom Shooter");
                command.setName(name);
                return command;
            };
        }

        public static final class OuterIntake {
            public static final Function<Double, Command> create = (speed) -> {
                Command command = Commands
                        .run(() -> RobotContainer.outerIntakeSubsystem.spin.accept(speed),
                                RobotContainer.outerIntakeSubsystem);
                String name = String.format("Run Outer Intake");
                command.setName(name);
                return command;
            };
        }

        public static final class InnerIntake {
            public static final Function<Double, Command> create = (speed) -> {
                Command command = Commands
                        .run(() -> RobotContainer.innerIntakeSubsystem.spin.accept(speed),
                                RobotContainer.innerIntakeSubsystem);
                String name = String.format("Run Inner Intake");
                command.setName(name);
                return command;
            };
        }

        public static final class Transport {
            public static final Function<Double, Command> create = (speed) -> {
                Command command = Commands
                        .run(() -> RobotContainer.transportSubsystem.spin.accept(speed),
                                RobotContainer.transportSubsystem);
                String name = String.format("Run Transport");
                command.setName(name);
                return command;
            };
        }

        public static final class Singulator {
            public static final Function<Double, Command> create = (speed) -> {
                Command command = Commands
                        .run(() -> RobotContainer.singulatorSubsystem.spin.accept(speed),
                                RobotContainer.singulatorSubsystem);
                String name = String.format("Run Singulator");
                command.setName(name);
                return command;
            };
        }
    }

    public BasicCommands() {
    }

}
