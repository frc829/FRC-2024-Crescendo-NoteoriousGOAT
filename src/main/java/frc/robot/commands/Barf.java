package frc.robot.commands;

import static frc.robot.RobotContainer.*;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public abstract class Barf {
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
                        singulatorSubsystem.createSetSpeedCommand(() -> Constants.singulatorPercent),
                        topShooterSubsystem.createSetSpeedCommand(() -> Constants.topShooterPercent),
                        bottomShooterSubsystem.createSetSpeedCommand(() -> Constants.bottomShooterPercent));
}