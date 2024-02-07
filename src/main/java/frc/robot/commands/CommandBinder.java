package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Value;

import java.util.function.Supplier;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Function;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PickupSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterTiltSubsystem;

public class CommandBinder {

        private static final class Constants {
                private static final class Pickup {
                        private static final DoubleSupplier outerSpeed = () -> 0.5;
                        private static final DoubleSupplier innerSpeed = () -> 0.5;
                        private static final DoubleSupplier transportSpeed = () -> 0.5;
                        private static final DoubleSupplier singulatorSpeed = () -> 0.0;
                        private static final DoubleSupplier topShooterSpeed = () -> 0.5;
                        private static final DoubleSupplier bottomShooterSpeed = () -> 0.5;
                        private static final Supplier<Measure<Angle>> tiltAngle = () -> Degrees.of(60.0);
                        private static final Supplier<Measure<Angle>> angleTolerance = () -> Degrees.of(1.0);
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
                                private static final Supplier<Measure<Angle>> tiltAngle = () -> Degrees.of(80.0);
                                private static final Supplier<Measure<Angle>> angleTolerance = () -> Degrees.of(1.0);

                        }
                }
        }

        private CommandBinder() {
        }

        private static final Function<PickupSubsystem, Function<ShooterSubsystem, Function<ShooterTiltSubsystem, Command>>> createPickupCommand = (
                        pickup) -> (shooter) -> (tilt) -> {
                                Command pickupCommand = Commands.either(
                                                Commands.parallel(
                                                                tilt.createHoldCommand.get(),
                                                                pickup.createStopCommand.get(),
                                                                shooter.createSpinShootersCommand
                                                                                .apply(Constants.Pickup.topShooterSpeed)
                                                                                .apply(Constants.Pickup.bottomShooterSpeed)),
                                                tilt.createTurnTiltCommand.apply(Constants.Pickup.tiltAngle)
                                                                .until(() -> MathUtil.isNear(
                                                                                Constants.Pickup.tiltAngle.get()
                                                                                                .in(Degrees),
                                                                                tilt.angle.in(Degrees),
                                                                                Constants.Pickup.angleTolerance.get()
                                                                                                .in(Degrees)))
                                                                .andThen(Commands.parallel(
                                                                                tilt.createHoldCommand.get(),
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
                                                                                                .apply(Constants.Pickup.bottomShooterSpeed))),

                                                pickup.hasNote);
                                pickupCommand.setName("Pickup");
                                return pickupCommand;
                        };

        private static final Function<PickupSubsystem, Function<ShooterSubsystem, Function<ShooterTiltSubsystem, Command>>> createFenderShootCommand = (
                        pickup) -> (shooter) -> (tilt) -> {

                                Command shootCommand = Commands.deadline(
                                                tilt.createTurnTiltCommand.apply(Constants.Shoot.Fender.tiltAngle)
                                                                .until(() -> MathUtil.isNear(
                                                                                Constants.Shoot.Fender.tiltAngle.get()
                                                                                                .in(Degrees),
                                                                                tilt.angle.in(Degrees),
                                                                                Constants.Shoot.Fender.angleTolerance
                                                                                                .get()
                                                                                                .in(Degrees))),
                                                shooter.createSpinShootersCommand
                                                                .apply(Constants.Shoot.Fender.topShooterSpeed)
                                                                .apply(Constants.Shoot.Fender.bottomShooterSpeed))
                                                .andThen(Commands.deadline(
                                                                shooter.createSpinShootersCommand
                                                                                .apply(Constants.Shoot.Fender.topShooterSpeed)
                                                                                .apply(Constants.Shoot.Fender.bottomShooterSpeed)
                                                                                .until(() -> MathUtil.isNear(
                                                                                                Constants.Shoot.Fender.topShooterSpeed
                                                                                                                .getAsDouble(),
                                                                                                shooter.topVelocity.in(
                                                                                                                Value),
                                                                                                Constants.Shoot.Fender.speedTolerance)),
                                                                tilt.createHoldCommand.get()))
                                                .andThen(Commands.waitSeconds(Constants.Shoot.shutOffTime)
                                                                .deadlineWith(
                                                                                tilt.createHoldCommand.get(),
                                                                                pickup.createPickupControlCommand
                                                                                                .apply(Constants.Shoot.Fender.outerSpeed)
                                                                                                .apply(Constants.Shoot.Fender.innerSpeed)
                                                                                                .apply(Constants.Shoot.Fender.transportSpeed)
                                                                                                .apply(Constants.Shoot.Fender.singulatorSpeed),
                                                                                shooter.createSpinShootersCommand
                                                                                                .apply(Constants.Shoot.Fender.topShooterSpeed)
                                                                                                .apply(Constants.Shoot.Fender.bottomShooterSpeed)));

                                shootCommand.setName("Fender Shoot");
                                return shootCommand;
                        };

        public static final Function<PickupSubsystem, Function<ShooterSubsystem, Function<ShooterTiltSubsystem, Consumer<Trigger>>>> bindManualPickupCommand = (
                        pickup) -> (shooter) -> (tilt) -> (trigger) -> {
                                Command manualPickupCommand = createPickupCommand
                                                .apply(pickup)
                                                .apply(shooter)
                                                .apply(tilt);
                                manualPickupCommand.setName("Manual Pickup");
                                trigger.whileTrue(manualPickupCommand);
                        };

        public static final Function<PickupSubsystem, Function<ShooterSubsystem, Function<ShooterTiltSubsystem, Consumer<Trigger>>>> bindManualFenderShootCommand = (
                        pickup) -> (shooter) -> (tilt) -> (trigger) -> {
                                Command manualShooterCommand = createFenderShootCommand
                                                .apply(pickup)
                                                .apply(shooter)
                                                .apply(tilt);
                                manualShooterCommand.setName("Manual Fender Shoot");
                                trigger.whileTrue(manualShooterCommand);
                        };

        public static final Function<ShooterTiltSubsystem, Function<DoubleSupplier, Consumer<Trigger>>> bindManualShooterTiltCommand = (
                        shooterTilt) -> (driveSpeed) -> (trigger) -> {
                                Command manualShooterTiltCommand = shooterTilt.createDriveTiltCommand
                                                .apply(driveSpeed);
                                manualShooterTiltCommand.setName("Manual Shooter Tilt Drive");
                                trigger.whileTrue(manualShooterTiltCommand);
                        };

        public static final Function<ElevatorSubsystem, Function<DoubleSupplier, Consumer<Trigger>>> bindManualElevatorCommand = (
                        elevator) -> (driveSpeed) -> (trigger) -> {
                                Command manualShooterTiltCommand = elevator.createDriveElevatorcommand
                                                .apply(driveSpeed);
                                manualShooterTiltCommand.setName("Manual Elevator Drive");
                                trigger.whileTrue(manualShooterTiltCommand);
                        };

}
