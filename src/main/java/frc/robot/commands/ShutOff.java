package frc.robot.commands;

import static frc.robot.RobotContainer.bottomShooterSubsystem;
import static frc.robot.RobotContainer.singulatorSubsystem;
import static frc.robot.RobotContainer.topShooterSubsystem;
import static frc.robot.RobotContainer.transportSubsystem;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public abstract class ShutOff {
        public static final Supplier<Command> shutOffCommand = () -> {
                return Commands.sequence(
                        topShooterSubsystem.createStopCommand(),
                        bottomShooterSubsystem.createStopCommand(),
                        Commands.runOnce(singulatorSubsystem.stop, singulatorSubsystem),
                        transportSubsystem.createStopCommand()
                );
        };
}