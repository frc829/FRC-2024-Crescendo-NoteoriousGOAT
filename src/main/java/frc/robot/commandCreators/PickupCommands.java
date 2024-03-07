package frc.robot.commandCreators;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static frc.robot.RobotContainer.telemetrySubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.fasterxml.jackson.databind.introspect.TypeResolutionContext.Basic;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.commandCreators.BasicCommands.Tilt;

public class PickupCommands {

        public static final class Level{
                               public static final Command level = ResetAndHoldingCommands.setElevatorTiltUntil
                                .apply(Meters.of(0.0))
                                .apply(Degrees.of(0.0)); 
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
        }

        private static final class Constants {
                private static final class Ground {
                        private static Measure<Angle> tiltAngle = Degrees.of(55);
                        private static Measure<Distance> elevatorPosition = Meters.of(0);
                        private static double transportPercent = 0.9;
                        private static double innerIntakePercent = 0.9;
                        private static double outerIntakePercent = 0.9;
                }

                private static final class BabyBird {
                        private static MutableMeasure<Angle> tiltAngle = MutableMeasure.ofRelativeUnits(55, Degrees);
                        private static MutableMeasure<Distance> elevatorPosition = MutableMeasure.ofBaseUnits(0.0,
                                        Meters);
                        private static double topShooterPercent = 0.2;
                        private static double bottomShooterPercent = -0.2;
                        private static double transportPercent = 0.0;
                        private static double singulatorPercent = 0.5;
                }

                private static final class Barf {
                        private static double topShooterPercent = -0.9;
                        private static double bottomShooterPercent = 0.9;
                        private static double singulatorPercent = -0.9;
                        private static double transportPercent = 0.9;
                        private static double innerIntakePercent = -0.9;
                        private static double outerIntakePercent = -0.9;
                }
        }

        public static final Supplier<Command> createGround = () -> {
                Command setElevatorAndTilt = Commands.parallel(
                                BasicCommands.Elevator.createSetElevatorPositionCommand
                                                .apply(Constants.Ground.elevatorPosition),
                                BasicCommands.Tilt.createSetTiltAngleCommand.apply(Constants.Ground.tiltAngle));

                Command elevatorHoldCommand = BasicCommands.HoldandStop.createForElevator.get();
                Command tiltHoldCommand = BasicCommands.HoldandStop.createForTilt.get();
                Command transportCommand = BasicCommands.Set.Transport.create.apply(Constants.Ground.transportPercent);
                Command singulatorCommand = BasicCommands.Set.Singulator.create.apply(0.0);
                Command innerIntakeCommand = BasicCommands.Set.InnerIntake.create
                                .apply(Constants.Ground.innerIntakePercent);
                Command outerIntakeCommand = BasicCommands.Set.OuterIntake.create
                                .apply(Constants.Ground.outerIntakePercent);

                Command groundPickupCommand = Commands
                                .parallel(elevatorHoldCommand, tiltHoldCommand, transportCommand, singulatorCommand,
                                                innerIntakeCommand,
                                                outerIntakeCommand)
                                .until(RobotContainer.notedLoadedSubsystem.hasNote);

                Command elevatorTiltCommand3 = ResetAndHoldingCommands.setElevatorTiltForever
                                .apply(Meters.of(0))
                                .apply(Degrees.of(0));

                Command hasNoteCommand = Commands.parallel(
                                elevatorTiltCommand3,
                                BasicCommands.Set.OuterIntake.create.apply(0.0),
                                BasicCommands.Set.InnerIntake.create.apply(0.0),
                                BasicCommands.Set.Transport.create.apply(0.0),
                                BasicCommands.Set.Singulator.create.apply(0.0));

                Command pickupCommand = Commands.sequence(elevatorTiltCommand,
                                groundPickupCommand, hasNoteCommand);
                Command noPickupCommand = Commands.none();
                Command command = Commands.either(noPickupCommand, pickupCommand,
                                RobotContainer.notedLoadedSubsystem.hasNote);
                command.setName("Ground Pickup");
                return command;
        };

        public static final Supplier<Command> createGroundNoLevel = () -> {
                Command elevatorTiltCommand = ResetAndHoldingCommands.setElevatorTiltUntil
                                .apply(Constants.Ground.elevatorPosition)
                                .apply(Constants.Ground.tiltAngle);

                Command elevatorHoldCommand = BasicCommands.HoldandStop.createForElevator.get();
                Command tiltHoldCommand = BasicCommands.HoldandStop.createForTilt.get();
                Command transportCommand = BasicCommands.Set.Transport.create.apply(Constants.Ground.transportPercent);
                Command singulatorCommand = BasicCommands.Set.Singulator.create.apply(0.0);
                Command innerIntakeCommand = BasicCommands.Set.InnerIntake.create
                                .apply(Constants.Ground.innerIntakePercent);
                Command outerIntakeCommand = BasicCommands.Set.OuterIntake.create
                                .apply(Constants.Ground.outerIntakePercent);

                Command groundPickupCommand = Commands
                                .parallel(elevatorHoldCommand, tiltHoldCommand, transportCommand, singulatorCommand,
                                                innerIntakeCommand,
                                                outerIntakeCommand)
                                .until(RobotContainer.notedLoadedSubsystem.hasNote);

                Command hasNoteCommand = Commands.parallel(
                                BasicCommands.Set.OuterIntake.create.apply(0.0),
                                BasicCommands.Set.InnerIntake.create.apply(0.0),
                                BasicCommands.Set.Transport.create.apply(0.0),
                                BasicCommands.Set.Singulator.create.apply(0.0));

                Command pickupCommand = Commands.sequence(elevatorTiltCommand,
                                groundPickupCommand, hasNoteCommand);
                Command noPickupCommand = Commands.none();
                Command command = Commands.either(noPickupCommand, pickupCommand,
                                RobotContainer.notedLoadedSubsystem.hasNote);
                command.setName("Ground Pickup");
                return command;
        };

        public static final Supplier<Command> createBabyBird = () -> {
                Command elevatorTiltCommand = ResetAndHoldingCommands.setElevatorTiltUntil
                                .apply(Constants.BabyBird.elevatorPosition)
                                .apply(Constants.BabyBird.tiltAngle);

                Command elevatorHoldCommand = BasicCommands.HoldandStop.createForElevator.get();
                Command tiltHoldCommand = BasicCommands.HoldandStop.createForTilt.get();
                Command transportCommand = BasicCommands.Set.Transport.create
                                .apply(Constants.BabyBird.transportPercent);
                Command singulatorCommand = BasicCommands.Set.Singulator.create
                                .apply(Constants.BabyBird.singulatorPercent);
                Command topShooterCommand = BasicCommands.Set.TopShooter.create
                                .apply(() -> Constants.BabyBird.topShooterPercent);
                Command bottomShooterCommand = BasicCommands.Set.BottomShooter.create
                                .apply(() -> Constants.BabyBird.bottomShooterPercent);

                Command babyBirdPickupCommand = Commands
                                .parallel(elevatorHoldCommand, tiltHoldCommand, transportCommand, singulatorCommand,
                                                topShooterCommand, bottomShooterCommand)
                                .until(RobotContainer.notedLoadedSubsystem.hasNote);

                Command elevatorTiltCommand3 = ResetAndHoldingCommands.setElevatorTiltForever
                                .apply(Meters.of(0))
                                .apply(Degrees.of(0));

                Command hasNoteCommand = Commands.parallel(
                                elevatorTiltCommand3,
                                BasicCommands.Set.OuterIntake.create.apply(0.0),
                                BasicCommands.Set.InnerIntake.create.apply(0.0),
                                BasicCommands.Set.Transport.create.apply(0.0),
                                BasicCommands.Set.Singulator.create.apply(0.0),
                                BasicCommands.Set.TopShooter.create.apply(() -> 0.0),
                                BasicCommands.Set.BottomShooter.create.apply(() -> 0.0));

                Command command = Commands.sequence(elevatorTiltCommand,
                                babyBirdPickupCommand, hasNoteCommand);
                command.setName("Baby Bird Pickup");
                return command;
        };

        public static final Supplier<Command> createBarf = () -> {
                Command topShooterCommand = BasicCommands.Set.TopShooter.create
                                .apply(() -> Constants.Barf.topShooterPercent);
                Command bottomShooterCommand = BasicCommands.Set.BottomShooter.create
                                .apply(() -> Constants.Barf.bottomShooterPercent);
                Command singulatorCommand = BasicCommands.Set.Singulator.create.apply(Constants.Barf.singulatorPercent);
                Command transportCommand = BasicCommands.Set.Transport.create.apply(Constants.Barf.transportPercent);
                Command innerIntakeCommand = BasicCommands.Set.InnerIntake.create
                                .apply(Constants.Barf.innerIntakePercent);
                Command outerIntakeCommand = BasicCommands.Set.OuterIntake.create
                                .apply(Constants.Barf.outerIntakePercent);

                Command command = Commands.parallel(
                                topShooterCommand,
                                bottomShooterCommand,
                                singulatorCommand,
                                transportCommand,
                                innerIntakeCommand,
                                outerIntakeCommand);
                command.setName("Barf");
                return command;
        };

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

                Command pickupDetectedNote = Commands.parallel(goToNoteCommand, createGround.get());

                Command command = Commands.sequence(setObjectDetectModeCommand, waitUntilNoteDetected,
                                pickupDetectedNote,
                                setFieldDetectModeCommand);

                command.setName("Note Detect");
                return command;
        };

        public PickupCommands() {
        }

}
