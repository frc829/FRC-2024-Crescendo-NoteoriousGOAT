package frc.robot.commandCreators;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static frc.robot.RobotContainer.telemetrySubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;

public class PickupCommands {

        public static final class Level {
                private static final class Constants {
                        private static final Measure<Angle> tiltAngle = Degrees.of(0.0);
                        private static final Measure<Distance> elevatorPosition = Meters.of(0.0);
                        public static final BooleanSupplier elevatorAndTiltAtPositionCondition = () -> {
                                return BasicCommands.Elevator.createAtPositionCondition.apply(elevatorPosition)
                                                .getAsBoolean() &&
                                                BasicCommands.Tilt.createAtAngleCondition.apply(tiltAngle)
                                                                .getAsBoolean();

                        };

                }

                public static final Supplier<Command> command = () -> {
                        return Commands.parallel(
                                        BasicCommands.Elevator.createSetElevatorPositionCommand
                                                        .apply(Constants.elevatorPosition),
                                        BasicCommands.Tilt.createSetTiltAngleCommand.apply(Constants.tiltAngle))
                                        .until(Constants.elevatorAndTiltAtPositionCondition);
                };
        }

        public static final class Ground {
                private static final class Constants {
                        private static Measure<Angle> tiltAngle = Degrees.of(55);
                        private static Measure<Distance> elevatorPosition = Meters.of(0);
                        private static double transportPercent = 0.9;
                        private static double innerIntakePercent = 0.9;
                        private static double outerIntakePercent = 0.9;
                        private static final BooleanSupplier elevatorAndTiltAtPositionCondition = () -> {
                                return BasicCommands.Elevator.createAtPositionCondition.apply(elevatorPosition)
                                                .getAsBoolean() &&
                                                BasicCommands.Tilt.createAtAngleCondition.apply(tiltAngle)
                                                                .getAsBoolean();

                        };
                }

                private static final Supplier<Command> elevatorAndTiltAtPositionCommand = () -> {
                        return Commands.parallel(
                                        BasicCommands.Elevator.createSetElevatorPositionCommand
                                                        .apply(Constants.elevatorPosition),
                                        BasicCommands.Tilt.createSetTiltAngleCommand.apply(Constants.tiltAngle))
                                        .until(Constants.elevatorAndTiltAtPositionCondition);
                };

                private static final Supplier<Command> intakeCommand = () -> {
                        return Commands.parallel(
                                        BasicCommands.Elevator.createHoldElevatorCommand.get(),
                                        BasicCommands.Tilt.createHoldTiltCommand.get(),
                                        BasicCommands.OuterIntake.createSpinCommand.apply(Constants.outerIntakePercent),
                                        BasicCommands.InnerIntake.createSpinCommand.apply(Constants.innerIntakePercent),
                                        BasicCommands.Transport.createSpinCommand.apply(Constants.transportPercent));
                };

                public static final Supplier<Command> groundCommand = () -> Commands
                                .sequence(elevatorAndTiltAtPositionCommand.get(), intakeCommand.get());

                public static final Supplier<Command> groundCommandThenLevel = () -> groundCommand.get()
                                .until(RobotContainer.notedLoadedSubsystem.hasNote)
                                .andThen(Level.command.get());
        }

        public static final class BabyBird {
                private static final class Constants {
                        private static MutableMeasure<Angle> tiltAngle = MutableMeasure.ofRelativeUnits(55, Degrees);
                        private static MutableMeasure<Distance> elevatorPosition = MutableMeasure.ofBaseUnits(0.0,
                                        Meters);
                        private static double topShooterPercent = 0.5;
                        private static double bottomShooterPercent = -0.5;
                        private static double singulatorPercent = 0.5;
                        private static final BooleanSupplier elevatorAndTiltAtPositionCondition = () -> {
                                return BasicCommands.Elevator.createAtPositionCondition.apply(elevatorPosition)
                                                .getAsBoolean() &&
                                                BasicCommands.Tilt.createAtAngleCondition.apply(tiltAngle)
                                                                .getAsBoolean();

                        };
                }

                private static final Supplier<Command> elevatorAndTiltAtPositionCommand = () -> {
                        return Commands.parallel(
                                        BasicCommands.Elevator.createSetElevatorPositionCommand
                                                        .apply(Constants.elevatorPosition),
                                        BasicCommands.Tilt.createSetTiltAngleCommand.apply(Constants.tiltAngle))
                                        .until(Constants.elevatorAndTiltAtPositionCondition);
                };

                private static final Supplier<Command> intakeCommand = () -> {
                        return Commands.parallel(
                                        BasicCommands.Elevator.createHoldElevatorCommand.get(),
                                        BasicCommands.Tilt.createHoldTiltCommand.get(),
                                        BasicCommands.Singulator.createSpinCommand.apply(Constants.singulatorPercent),
                                        BasicCommands.TopShooter.createSpinCommand.apply(Constants.topShooterPercent),
                                        BasicCommands.BottomShooter.createSpinCommand
                                                        .apply(Constants.bottomShooterPercent));
                };

                private static final Supplier<Command> babyBirdCommand = () -> Commands
                                .sequence(elevatorAndTiltAtPositionCommand.get(), intakeCommand.get());

                public static final Supplier<Command> babyBirdCommandThenLevel = () -> babyBirdCommand.get()
                                .until(RobotContainer.notedLoadedSubsystem.hasNoteBB)
                                .andThen(Level.command.get());
        }

        public static final class Barf {
                private static final class Constants {
                        private static double topShooterPercent = -0.9;
                        private static double bottomShooterPercent = 0.9;
                        private static double singulatorPercent = -0.9;
                        private static double transportPercent = 0.9;
                        private static double innerIntakePercent = -0.9;
                        private static double outerIntakePercent = -0.9;
                }

                public static final Supplier<Command> barfCommand = () -> Commands.parallel(
                                BasicCommands.OuterIntake.createSpinCommand.apply(Constants.outerIntakePercent),
                                BasicCommands.InnerIntake.createSpinCommand.apply(Constants.innerIntakePercent),
                                BasicCommands.Transport.createSpinCommand.apply(Constants.transportPercent),
                                BasicCommands.Singulator.createSpinCommand.apply(Constants.singulatorPercent),
                                BasicCommands.TopShooter.createSpinCommand.apply(Constants.topShooterPercent),
                                BasicCommands.BottomShooter.createSpinCommand
                                                .apply(Constants.bottomShooterPercent));
        }

        public static final Supplier<Command> createNoteDetect = () -> {
                Command setObjectDetectModeCommand = Commands.runOnce(
                                RobotContainer.telemetrySubsystem.enableObjectDetectors.get(0)::run,
                                RobotContainer.telemetrySubsystem);

                Command waitUntilNoteDetected = Commands.waitUntil(() -> {
                        return telemetrySubsystem.objectPositions.get(0).getSecond().get().isPresent();
                });

                Command goToNoteCommand = Commands.deferredProxy(DriveCommands.goToNoteCommandSupplier);

                Command setFieldDetectModeCommand = Commands.runOnce(
                                RobotContainer.telemetrySubsystem.enableFieldDetectors.get(0)::run,
                                RobotContainer.telemetrySubsystem);

                Command pickupDetectedNote = Commands.parallel(goToNoteCommand, Ground.groundCommandThenLevel.get());

                Command command = Commands.sequence(setObjectDetectModeCommand, waitUntilNoteDetected,
                                pickupDetectedNote,
                                setFieldDetectModeCommand);

                command.setName("Note Detect");
                return command;
        };

        public PickupCommands() {
        }

}
