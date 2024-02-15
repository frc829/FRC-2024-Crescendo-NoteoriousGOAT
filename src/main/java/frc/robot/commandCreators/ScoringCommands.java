package frc.robot.commandCreators;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import java.util.function.Supplier;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ScoringCommands {

        private static final class Constants {
                private static final class Amp {
                        private static final Measure<Angle> tiltAngle = Degrees.of(75);
                        private static final Measure<Distance> elevatorPosition = Meters.of(0.0);
                        private static final double topShooterPercent = 0.5;
                        private static final double bottomShooterPercent = 0.5;
                        private static final double transportPercent = 0.5;
                        private static final double singulatorPercent = 0.5;
                }

                private static final class Fender {
                        private static final Measure<Angle> tiltAngle = Degrees.of(75);
                        private static final Measure<Distance> elevatorPosition = Meters.of(0.5);
                        private static final double topShooterPercent = 0.5;
                        private static final double bottomShooterPercent = 0.5;
                        private static final double transportPercent = 0.0;
                        private static final double singulatorPercent = 0.5;
                }

                private static final class Ranged {
                        private static final double singulatorPercent = 0.0;
                        private static final double transportPercent = 0.0;
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
                Command topShooterCommand = BasicCommands.Set.TopShooter.create.apply(() -> Constants.Amp.topShooterPercent);
                Command bottomShooterCommand = BasicCommands.Set.BottomShooter.create.apply(() -> Constants.Amp.bottomShooterPercent);

                Command ampCommands = Commands
                                .parallel(elevatorHoldCommand, tiltHoldCommand, transportCommand, singulatorCommand,
                                                topShooterCommand, bottomShooterCommand);

                Command command = Commands.sequence(elevatorTiltCommand,
                                ampCommands);
                command.setName("Amp Score");
                return command;
        };

        public ScoringCommands() {
        }

}
