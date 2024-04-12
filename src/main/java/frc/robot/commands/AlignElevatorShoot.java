package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static frc.robot.RobotContainer.*;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public abstract class AlignElevatorShoot {
        private static final class Constants {
                private static Measure<Angle> tiltAngle = Degrees.of(31);
                private static Measure<Distance> elevatorPosition = Meters.of(0.30);
                private static double topShooterPercent = -0.7;
                private static double bottomShooterPercent = 0.7;
                private static double transportPercent = 0.9;
                private static double singulatorPercent = -0.9;
                private static final double shooterTolerancePercent = 0.10;
                private static final double endOfShootDelay = 0.2;

                private static final double tolerance = 2;
                private static final PIDController rotationPIDController = new PIDController(0.10, 0, 0);

        }

        private static final BooleanSupplier shootersAtSpeed = () -> {
                boolean topCondition = topShooterSubsystem.atSpeed(Constants.topShooterPercent,
                                Constants.shooterTolerancePercent);
                boolean bottomCondition = bottomShooterSubsystem.atSpeed(Constants.bottomShooterPercent,
                                Constants.shooterTolerancePercent);
                boolean condition = topCondition && bottomCondition;
                SmartDashboard.putBoolean("ShooterAt Speed", condition);
                return condition;
        };

        private static final BooleanSupplier elevatorAndTiltAtPositionCondition = () -> {
                return BasicCommands.Elevator.createAtPositionCondition.apply(Constants.elevatorPosition)
                                .getAsBoolean() &&
                                BasicCommands.Tilt.createAtAngleCondition.apply(Constants.tiltAngle)
                                                .getAsBoolean();

        };

        private static final Supplier<Command> createAimCommand = () -> {
                Command command = Commands.parallel(
                                BasicCommands.Elevator.createSetAndHoldElevatorPositionCommand
                                                .apply(Constants.elevatorPosition),
                                BasicCommands.Tilt.createSetAndHoldTiltAngleCommand.apply(Constants.tiltAngle),
                                topShooterSubsystem.createSetVelocityRatioCommand(() -> Constants.topShooterPercent),
                                bottomShooterSubsystem
                                                .createSetVelocityRatioCommand(() -> Constants.bottomShooterPercent))
                                .until(elevatorAndTiltAtPositionCondition);
                command.setName("Fender Aim");
                return command;
        };

        private static final Supplier<Command> createSpinUp = () -> {
                Command command = Commands.parallel(
                                BasicCommands.Elevator.createSetAndHoldElevatorPositionCommand
                                                .apply(Constants.elevatorPosition),
                                BasicCommands.Tilt.createSetAndHoldTiltAngleCommand.apply(Constants.tiltAngle),
                                topShooterSubsystem.createSetVelocityRatioCommand(() -> Constants.topShooterPercent),
                                bottomShooterSubsystem
                                                .createSetVelocityRatioCommand(() -> Constants.bottomShooterPercent))
                                .until(shootersAtSpeed);
                command.setName("Fender Spin Up");
                return command;
        };

        private static final Supplier<Command> createShootCommand = () -> {
                Command command = Commands.parallel(
                                BasicCommands.Elevator.createSetAndHoldElevatorPositionCommand
                                                .apply(Constants.elevatorPosition),
                                BasicCommands.Tilt.createSetAndHoldTiltAngleCommand.apply(Constants.tiltAngle),
                                topShooterSubsystem.createSetVelocityRatioCommand(() -> Constants.topShooterPercent),
                                bottomShooterSubsystem
                                                .createSetVelocityRatioCommand(() -> Constants.bottomShooterPercent),
                                singulatorSubsystem.createSetVelocityRatioCommand(() -> Constants.singulatorPercent),
                                transportSubsystem.createSetVelocityRatioCommand(() -> Constants.transportPercent));
                command.setName("Fender Shoot");
                return command;
        };

        private static final Runnable rotateDriveToFoundTarget = () -> {
                var angle = telemetrySubsystem.poseEstimate.get().getRotation().getDegrees();
                double omegaRadiansPerSecond = Constants.rotationPIDController.calculate(angle);
                ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, omegaRadiansPerSecond);
                driveSubsystem.controlRobotChassisSpeeds.apply(new Translation2d())
                                .accept(chassisSpeeds);
        };

        private static final Supplier<Command> rotateDriveToAlignTargetCommand = () -> {
                Command command = Commands.run(rotateDriveToFoundTarget, driveSubsystem);
                command.setName("Rotate To Align Target");
                return command;
        };

        private static final BooleanSupplier setDistanceAndAngle = () -> {
                var distance = telemetrySubsystem.priorityTargetDistance.get();
                var angle = telemetrySubsystem.priorityTargetRotation.get();
                if (distance.isPresent() && angle.isPresent()) {
                        SmartDashboard.putBoolean("Has Priority Tag", true);
                        Rotation2d poseAngle = telemetrySubsystem.poseEstimate.get().getRotation();
                        poseAngle = poseAngle.plus(angle.get());
                        if (DriverStation.getAlliance().isPresent()
                                        && DriverStation.getAlliance().get() == Alliance.Red) {
                                Constants.rotationPIDController.setSetpoint(poseAngle.getDegrees());
                        } else {
                                Constants.rotationPIDController.setSetpoint(poseAngle.getDegrees());
                        }
                        Constants.rotationPIDController.setTolerance(Constants.tolerance);
                        return true;
                } else {
                        SmartDashboard.putBoolean("Has Priority Tag", false);
                        return false;
                }
        };

        private static final Supplier<Command> setDistanceAndAngleCommand = () -> {
                Runnable blank = () -> {
                };
                return Commands.run(blank).until(setDistanceAndAngle);
        };

        private static final Supplier<Command> createAimCommandInitial = () -> {
                Command command = Commands.parallel(
                                rotateDriveToAlignTargetCommand.get(),
                                BasicCommands.Elevator.createSetAndHoldElevatorPositionCommand
                                                .apply(Constants.elevatorPosition),
                                BasicCommands.Tilt.createSetAndHoldTiltAngleCommand.apply(Constants.tiltAngle),
                                topShooterSubsystem.createSetVelocityRatioCommand(() -> Constants.topShooterPercent),
                                bottomShooterSubsystem
                                                .createSetVelocityRatioCommand(() -> Constants.bottomShooterPercent))
                                .until(() -> Constants.rotationPIDController.atSetpoint());
                command.setName("Ranged Aim Initial");
                return command;
        };

        public static final Supplier<Command> create = () -> {
                return Commands.sequence(
                                TelemetryCommands.createSetRearCameraToFieldCommand.get(),
                                setDistanceAndAngleCommand.get(),
                                createAimCommandInitial.get(),
                                Commands.race(Commands.waitSeconds(0.75),
                                                Commands.run(driveSubsystem.stop, driveSubsystem)),
                                createAimCommand.get(),
                                createShootCommand.get());
        };

        public static final Supplier<Command> createWithDelay = () -> {
                return Commands.sequence(
                                createAimCommand.get(),
                                createSpinUp.get(),
                                createShootCommand.get()
                                                .raceWith(Commands.waitSeconds(Constants.endOfShootDelay)),
                                singulatorSubsystem.createStopCommand());
        };
}