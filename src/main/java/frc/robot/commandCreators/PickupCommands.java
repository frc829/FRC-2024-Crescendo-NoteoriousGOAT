package frc.robot.commandCreators;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import java.util.function.Supplier;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;

public class PickupCommands implements Sendable {

        private static final class Constants {
                private static final class Ground {
                        private static final Measure<Angle> tiltAngle = Degrees.of(75);
                        private static final Measure<Distance> elevatorPosition = Meters.of(0.0);
                        private static final double transportPercent = 0.5;
                        private static final double innerIntakePercent = 0.5;
                        private static final double outerIntakePercent = 0.5;
                }

                private static final class BabyBird {
                        private static final Measure<Angle> tiltAngle = Degrees.of(75);
                        private static final Measure<Distance> elevatorPosition = Meters.of(0.5);
                        private static final double topShooterPercent = 0.5;
                        private static final double bottomShooterPercent = 0.5;
                        private static final double transportPercent = 0.0;
                        private static final double singulatorPercent = 0.5;
                }

                private static final class Barf {
                        private static final double topShooterPercent = 0.0;
                        private static final double bottomShooterPercent = 0.0;
                        private static final double singulatorPercent = 0.0;
                        private static final double transportPercent = 0.0;
                        private static final double innerIntakePercent = 0.0;
                        private static final double outerIntakePercent = 0.0;
                }
        }

        public static final Supplier<Command> createGround = () -> {
                Command elevatorTiltCommand = ResetAndHoldingCommands.setElevatorTiltUntil
                                .apply(Constants.Ground.elevatorPosition)
                                .apply(Constants.Ground.tiltAngle);

                Command elevatorHoldCommand = BasicCommands.HoldandStop.createForElevator.get();
                Command tiltHoldCommand = BasicCommands.HoldandStop.createForTilt.get();
                Command transportCommand = BasicCommands.Set.Transport.create.apply(Constants.Ground.transportPercent);
                Command innerIntakeCommand = BasicCommands.Set.InnerIntake.create
                                .apply(Constants.Ground.innerIntakePercent);
                Command outerIntakeCommand = BasicCommands.Set.OuterIntake.create
                                .apply(Constants.Ground.outerIntakePercent);

                Command groundPickupCommand = Commands
                                .parallel(elevatorHoldCommand, tiltHoldCommand, transportCommand, innerIntakeCommand,
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
                                BasicCommands.Set.Singulator.create.apply(0.0),
                                BasicCommands.Set.TopShooter.create.apply(() -> 0.0),
                                BasicCommands.Set.BottomShooter.create.apply(() -> 0.0));

                Command command = Commands.sequence(elevatorTiltCommand,
                                groundPickupCommand, hasNoteCommand);
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
                Command setObjectDetectModeCommand = Commands.runOnce(RobotContainer.telemetrySubsystem.enableObjectDetectors.get(0)::run,
                                RobotContainer.telemetrySubsystem);

                Command goToNoteCommand = Commands.deferredProxy(DriveCommands.goToNoteCommandSupplier.get());

                Command setFieldDetectModeCommand = Commands.runOnce(RobotContainer.telemetrySubsystem.enableFieldDetectors.get(0)::run,
                                RobotContainer.telemetrySubsystem);
                
                Command pickupDetectedNote = Commands.parallel(goToNoteCommand, createGround.get());

                Command command = Commands.sequence(setObjectDetectModeCommand, pickupDetectedNote, setFieldDetectModeCommand);
                
                command.setName("Note Detect");
                return command;
        };

        public PickupCommands() {
        }

        @Override
        public void initSendable(SendableBuilder builder) {

        }
}
