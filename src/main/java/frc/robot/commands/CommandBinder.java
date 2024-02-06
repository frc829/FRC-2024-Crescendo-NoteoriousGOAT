package frc.robot.commands;

import static edu.wpi.first.units.Units.Value;

import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Function;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.PickupSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class CommandBinder {

        private static final class Constants {
                private static final class Pickup {
                        private static final DoubleSupplier outerSpeed = () -> 0.5;
                        private static final DoubleSupplier innerSpeed = () -> 0.5;
                        private static final DoubleSupplier transportSpeed = () -> 0.5;
                        private static final DoubleSupplier singulatorSpeed = () -> 0.0;
                        private static final DoubleSupplier topShooterSpeed = () -> 0.5;
                        private static final DoubleSupplier bottomShooterSpeed = () -> 0.5;
                }

                private static final class Shoot {
                        private static final double shutOffTime = 5;

                        private static final class Fender {
                                private static final DoubleSupplier outerSpeed = () -> 0.0;
                                private static final DoubleSupplier innerSpeed = () -> 0.0;
                                private static final DoubleSupplier transportSpeed = () -> 0.5;
                                private static final DoubleSupplier singulatorSpeed = () -> 0.5;
                                private static final DoubleSupplier topShooterSpeed = () -> 0.5;
                                private static final DoubleSupplier bottomShooterSpeed = () -> 0.5;
                                private static final double speedTolerance = 0.05;
                        }
                }
        }

        private CommandBinder() {
        }

        public static final Function<PickupSubsystem, Function<ShooterSubsystem, Command>> createPickupCommand = (
                        pickup) -> (shooter) -> {
                                Command pickupCommand = Commands.either(
                                                Commands.parallel(
                                                                pickup.createStopCommand.get(),
                                                                shooter.createSpinShootersCommand
                                                                                .apply(Constants.Pickup.topShooterSpeed)
                                                                                .apply(Constants.Pickup.bottomShooterSpeed)),
                                                Commands.parallel(
                                                                pickup.createPickupControlCommand
                                                                                .apply(Constants.Pickup.outerSpeed)
                                                                                .apply(Constants.Pickup.innerSpeed)
                                                                                .apply(Constants.Pickup.transportSpeed)
                                                                                .apply(Constants.Pickup.singulatorSpeed)
                                                                                .until(pickup.hasNote)
                                                                                .andThen(pickup.createStopCommand
                                                                                                .get()),
                                                                shooter.createSpinShootersCommand
                                                                                .apply(Constants.Pickup.topShooterSpeed)
                                                                                .apply(Constants.Pickup.bottomShooterSpeed)),
                                                pickup.hasNote);
                                pickupCommand.setName("Pickup");
                                return pickupCommand;
                        };

        public static final Function<PickupSubsystem, Function<ShooterSubsystem, Command>> createFenderShootCommand = (
                        pickup) -> (shooter) -> {

                                Command shootCommand = shooter.createSpinShootersCommand
                                                .apply(Constants.Shoot.Fender.topShooterSpeed)
                                                .apply(Constants.Shoot.Fender.bottomShooterSpeed)
                                                .until(() -> MathUtil.isNear(
                                                                Constants.Shoot.Fender.topShooterSpeed.getAsDouble(),
                                                                shooter.topVelocity.in(Value),
                                                                Constants.Shoot.Fender.speedTolerance))
                                                .andThen(Commands.waitSeconds(Constants.Shoot.shutOffTime)
                                                                .deadlineWith(
                                                                                pickup.createPickupControlCommand
                                                                                                .apply(Constants.Shoot.Fender.outerSpeed)
                                                                                                .apply(Constants.Shoot.Fender.innerSpeed)
                                                                                                .apply(Constants.Shoot.Fender.transportSpeed)
                                                                                                .apply(Constants.Shoot.Fender.singulatorSpeed),
                                                                                shooter.createSpinShootersCommand
                                                                                                .apply(Constants.Shoot.Fender.topShooterSpeed)
                                                                                                .apply(Constants.Shoot.Fender.bottomShooterSpeed)));

                                shootCommand.setName("Shoot");
                                return shootCommand;
                        };

        public static final Function<PickupSubsystem, Function<ShooterSubsystem, Consumer<Trigger>>> bindManualPickupCommand = (
                        pickup) -> (shooter) -> (trigger) -> {
                                Command manualPickupCommand = createPickupCommand
                                                .apply(pickup)
                                                .apply(shooter);
                                manualPickupCommand.setName("Manual Pickup");
                                trigger.whileTrue(manualPickupCommand);
                        };

        public static final Function<PickupSubsystem, Function<ShooterSubsystem, Consumer<Trigger>>> bindManualFenderShootCommand = (
                        pickup) -> (shooter) -> (trigger) -> {
                                Command manualShooterCommand = createFenderShootCommand
                                                .apply(pickup)
                                                .apply(shooter);
                                manualShooterCommand.setName("Manual Shoot");
                                trigger.whileTrue(manualShooterCommand);
                        };

}
