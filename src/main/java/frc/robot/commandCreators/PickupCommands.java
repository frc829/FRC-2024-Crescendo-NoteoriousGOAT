package frc.robot.commandCreators;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
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
            private static final double topShooterPercent = 0.0;
            private static final double bottomShooterPercent = 0.0;
            private static final double transportPercent = 0.0;
            private static final double innerIntakePercent = 0.0;
            private static final double outerIntakePercent = 0.0;
        }

        private static final class BabyBird {

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
        Command elevatorCommand = BasicCommands.Set.ElevatorPosition.create.apply(Constants.Ground.elevatorPosition);
        Command tiltCommand = BasicCommands.Set.TiltAngle.create.apply(Constants.Ground.tiltAngle);

        BooleanSupplier elevatorTiltEndCondition = () -> {
            boolean elevatorAtPosition = MathUtil.isNear(
                    Constants.Ground.elevatorPosition.in(Meters),
                    RobotContainer.elevatorSubsystem.position.in(Meters),
                    BasicCommands.Set.ElevatorPosition.tolerance.in(Meters));
            boolean tiltAtPosition = MathUtil.isNear(
                    Constants.Ground.tiltAngle.in(Degrees),
                    RobotContainer.shooterTiltSubsystem.angle.in(Degrees),
                    BasicCommands.Set.TiltAngle.tolerance.in(Degrees));
            return elevatorAtPosition && tiltAtPosition;
        };
        Command elevatorTiltSetCommand = Commands.parallel(elevatorCommand, tiltCommand)
                .until(elevatorTiltEndCondition);

        Command transportCommand = BasicCommands.Set.Transport.create.apply(Constants.Ground.transportPercent);
        Command innerIntakeCommand = BasicCommands.Set.InnerIntake.create.apply(Constants.Ground.innerIntakePercent);
        Command outerIntakeCommand = BasicCommands.Set.OuterIntake.create.apply(Constants.Ground.outerIntakePercent);

        Command groundPickupCommand = Commands.parallel(transportCommand, innerIntakeCommand, outerIntakeCommand)
                .until(RobotContainer.notedLoadedSubsystem.hasNote);
        Command command = Commands.sequence(elevatorTiltSetCommand, groundPickupCommand);
        command.setName("Ground Pickup");
        return command;
    };

    public static final Supplier<Command> createBabyBird = () -> {
        return null;
    };

    public static final Supplier<Command> createBarf = () -> {
        Command topShooterCommand = BasicCommands.Set.TopShooter.create.apply(() -> Constants.Barf.topShooterPercent);
        Command bottomShooterCommand = BasicCommands.Set.BottomShooter.create
                .apply(() -> Constants.Barf.bottomShooterPercent);
        Command singulatorCommand = BasicCommands.Set.Singulator.create.apply(Constants.Barf.singulatorPercent);
        Command transportCommand = BasicCommands.Set.Transport.create.apply(Constants.Barf.transportPercent);
        Command innerIntakeCommand = BasicCommands.Set.InnerIntake.create.apply(Constants.Barf.innerIntakePercent);
        Command outerIntakeCommand = BasicCommands.Set.OuterIntake.create.apply(Constants.Barf.outerIntakePercent);

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

    public PickupCommands() {
    }

    @Override
    public void initSendable(SendableBuilder builder) {

    }
}
