package frc.robot.commandCreators;

import java.util.function.Supplier;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class PickupCommands implements Sendable {

    private static final class Constants {
        private static final class Ground {

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
        return null;
    };

    public static final Supplier<Command> createBabyBird = () -> {
        return null;
    };

    public static final Supplier<Command> createBarf = () -> {
        Command topShooterCommand = BasicsCommands.Set.TopShooter.create.apply(() -> Constants.Barf.topShooterPercent);
        Command bottomShooterCommand = BasicsCommands.Set.BottomShooter.create
                .apply(() -> Constants.Barf.bottomShooterPercent);
        Command singulatorCommand = BasicsCommands.Set.Singulator.create.apply(Constants.Barf.singulatorPercent);
        Command transportCommand = BasicsCommands.Set.Transport.create.apply(Constants.Barf.transportPercent);
        Command innerIntakeCommand = BasicsCommands.Set.InnerIntake.create.apply(Constants.Barf.innerIntakePercent);
        Command outerIntakeCommand = BasicsCommands.Set.OuterIntake.create.apply(Constants.Barf.outerIntakePercent);

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
