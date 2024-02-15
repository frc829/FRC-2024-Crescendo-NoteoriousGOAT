package frc.robot.commandCreators;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import java.util.function.BooleanSupplier;
import java.util.function.Function;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;

public class ElevatorTiltCommand {

    private static final class Constants {
    }

    public static final Function<Measure<Distance>, Function<Measure<Angle>, Command>> setUntil = (
            position) -> (angle) -> {
                Command elevatorCommand = BasicCommands.Set.ElevatorPosition.create.apply(position);
                Command tiltCommand = BasicCommands.Set.TiltAngle.create.apply(angle);

                BooleanSupplier elevatorAtPosition = () -> {
                    return MathUtil.isNear(
                            position.in(Meters),
                            RobotContainer.elevatorSubsystem.position.in(Meters),
                            BasicCommands.Set.ElevatorPosition.tolerance.in(Meters));
                };

                BooleanSupplier tiltAtPosition = () -> {
                    return MathUtil.isNear(
                            angle.in(Degrees),
                            RobotContainer.shooterTiltSubsystem.angle.in(Degrees),
                            BasicCommands.Set.TiltAngle.tolerance.in(Degrees));
                };

                BooleanSupplier elevatorTiltEndCondition = () -> {
                    return elevatorAtPosition.getAsBoolean() && tiltAtPosition.getAsBoolean();
                };

                Command command = Commands.parallel(elevatorCommand, tiltCommand).until(elevatorTiltEndCondition);
                command.setName("Set Elevator Tilt Until");
                return command;
            };

    public static final Function<Measure<Distance>, Function<Measure<Angle>, Command>> setForever = (
            position) -> (angle) -> {
                Command elevatorCommand = BasicCommands.Set.ElevatorPosition.create.apply(position);
                Command tiltCommand = BasicCommands.Set.TiltAngle.create.apply(angle);

                Command command = Commands.parallel(elevatorCommand, tiltCommand);
                command.setName("Set Elevator Tilt Forever");
                return command;
            };

    public ElevatorTiltCommand() {
    }
}
