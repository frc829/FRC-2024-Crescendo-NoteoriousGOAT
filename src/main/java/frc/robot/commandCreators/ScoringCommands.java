package frc.robot.commandCreators;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import java.util.function.Supplier;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

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

                Command ampCommands = Commands
                                .parallel(elevatorHoldCommand, tiltHoldCommand, transportCommand, singulatorCommand,
                                                topShooterCommand, bottomShooterCommand);

                Command command = Commands.sequence(elevatorTiltCommand,
                                ampCommands);
                command.setName("Fender Score");
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
