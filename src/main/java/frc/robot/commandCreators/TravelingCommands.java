package frc.robot.commandCreators;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import java.util.function.Supplier;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class TravelingCommands {

    private static final class Constants {
        private static final Measure<Angle> tiltAngle = Degrees.of(0);
        private static final Measure<Distance> elevatorPosition = Meters.of(0.0);
    }

    public static final Supplier<Command> createTravel = () -> {
        Command elevatorCommand = BasicCommands.Set.ElevatorPosition.create.apply(Constants.elevatorPosition);
        Command tiltCommand = BasicCommands.Set.TiltAngle.create.apply(Constants.tiltAngle);

        Command command = Commands.parallel(elevatorCommand, tiltCommand);
        command.setName("Travel");
        return command;
    };

    public TravelingCommands() {
    }
}
