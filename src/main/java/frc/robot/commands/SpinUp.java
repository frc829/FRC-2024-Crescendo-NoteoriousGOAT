package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import static frc.robot.RobotContainer.*;

public abstract class SpinUp {
        private static final class Constants {
                private static double topShooterPercent = -0.7;
                private static double bottomShooterPercent = 0.7;
        }

        public static final Supplier<Command> createSpinUp = () -> {
                Command command = Commands.parallel(
                        topShooterSubsystem.createSetSpeedCommand(() -> Constants.topShooterPercent),
                        bottomShooterSubsystem.createSetSpeedCommand(() -> Constants.bottomShooterPercent));
                command.setName("Spin");
                return command;
        };
}