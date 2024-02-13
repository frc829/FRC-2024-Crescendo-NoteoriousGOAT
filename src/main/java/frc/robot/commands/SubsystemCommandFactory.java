package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Millimeters;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;

public class SubsystemCommandFactory {

    private static final class Constants {
        private static final Translation2d speakerBlue = new Translation2d(
                null,
                null);

        private static final Translation2d speakerRed = new Translation2d(
                null,
                null);
    }

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
                                RobotContainer.elevatorSubsystem)
                        .until(() -> MathUtil.isNear(
                                position.in(Meters),
                                RobotContainer.elevatorSubsystem.position.in(Meters),
                                tolerance.in(Meters)))
                        .andThen(HoldandStop.createForElevator.get());
                String name = String.format("Move Elevator to %s m", position.in(Meters));
                command.setName(name);
                return command;
            };
        }

        public static final class TiltAngle {
            public static final Measure<Angle> tolerance = Degrees.of(0.5);
            public static final Function<Measure<Angle>, Command> create = (angle) -> {
                Command command = Commands
                        .run(() -> RobotContainer.shooterTiltSubsystem.turn.accept(angle),
                                RobotContainer.shooterTiltSubsystem)
                        .until(() -> MathUtil.isNear(
                                angle.in(Degrees),
                                RobotContainer.shooterTiltSubsystem.angle.in(Degrees),
                                tolerance.in(Degrees)))
                        .andThen(HoldandStop.createForTilt.get());
                String name = String.format("Move Tilt to %s degrees", angle.in(Degrees));
                command.setName(name);
                return command;
            };
        }

        public static final class TopShooter {
            public static final Function<DoubleSupplier, Command> create = (speed) -> {
                Command command = Commands
                        .run(() -> RobotContainer.topShooterSubsystem.spin.accept(speed.getAsDouble()));
                String name = String.format("Accelerate Top Shooter");
                command.setName(name);
                return command;
            };
        }

        public static final class BottomShooter {
            public static final Function<DoubleSupplier, Command> create = (speed) -> {
                Command command = Commands
                        .run(() -> RobotContainer.bottomShooterSubsystem.spin.accept(speed.getAsDouble()));
                String name = String.format("Accelerate Bottom Shooter");
                command.setName(name);
                return command;
            };
        }

        public static final class OuterIntake {
            public static final Function<Double, Command> create = (speed) -> {
                Command command = Commands
                        .run(() -> RobotContainer.outerIntakeSubsystem.spin.accept(speed));
                String name = String.format("Run Outer Intake");
                command.setName(name);
                return command;
            };
        }

        public static final class InnerIntake {
            public static final Function<Double, Command> create = (speed) -> {
                Command command = Commands
                        .run(() -> RobotContainer.innerIntakeSubsystem.spin.accept(speed));
                String name = String.format("Run Inner Intake");
                command.setName(name);
                return command;
            };
        }

        public static final class Transport {
            public static final Function<Double, Command> create = (speed) -> {
                Command command = Commands
                        .run(() -> RobotContainer.transportSubsystem.spin.accept(speed));
                String name = String.format("Run Transport");
                command.setName(name);
                return command;
            };
        }

        public static final class Singulator {
            public static final Function<Double, Command> create = (speed) -> {
                Command command = Commands
                        .run(() -> RobotContainer.singulatorSubsystem.spin.accept(speed));
                String name = String.format("Run Singulator");
                command.setName(name);
                return command;
            };
        }
    }

    public SubsystemCommandFactory() {
    }



}
