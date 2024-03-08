package frc.robot.commandCreators;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Value;

import java.util.function.BooleanSupplier;
import java.util.function.Function;
import java.util.function.Supplier;

import com.utility.Spline;
import com.utility.Spline.MonotoneCubicSpline;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import static frc.robot.RobotContainer.*;

public class AdvancedScoringCommands {
    public static final class Ranged {
        private static final class Constants {
            private static Measure<Distance> elevatorPosition = Meters.of(0.0);
            private static double topShooterPercent = -0.7;
            private static double bottomShooterPercent = 0.7;
            private static double transportPercent = 0.9;
            private static double singulatorPercent = -0.9;
            private static final double shooterTolerancePercent = 0.10;
            private static final double endOfShootDelay = 0.2;
            private static final double findTargetOmegaRadiansPerSecond = 2.0;

            private static final double tolerance = 0.5;
            private static final PIDController rotationPIDController = new PIDController(0.05, 0, 0);
            private static final double[] distances = new double[] {
                    1.28,
                    2.22,
                    2.87,
                    4.29,
                    4.8514
            };
            private static final double[] anglesDegrees = new double[] {
                    55.0,
                    42.0,
                    35.0,
                    30.0,
                    29.0
            };
            private static final Spline spline = MonotoneCubicSpline.createMonotoneCubicSpline(distances,
                    anglesDegrees);

            private static final Function<Measure<Distance>, Measure<Angle>> distanceToAngle = (distance) -> {
                return Degrees.of(spline.interpolate(distance.in(Meters)));
            };
        }

        private static final MutableMeasure<Distance> targetDistance = MutableMeasure.zero(Meters);
        private static final MutableMeasure<Angle> tiltAngle = MutableMeasure.zero(Degrees);

        private static final BooleanSupplier hasTarget = () -> {
            var targetDistance = telemetrySubsystem.priorityTargetDistance.get();
            var targetAngle = telemetrySubsystem.priorityTargetRotation.get();
            boolean condition = targetDistance.isPresent() && targetAngle.isPresent();
            SmartDashboard.putBoolean("Has Target", condition);
            return condition;
        };

        private static final BooleanSupplier shootersAtSpeed = () -> {
            boolean topCondition = MathUtil.isNear(
                    Math.abs(Constants.topShooterPercent),
                    Math.abs(topShooterSubsystem.velocity.in(Value)),
                    Constants.shooterTolerancePercent);
            boolean bottomCondition = MathUtil.isNear(
                    Math.abs(Constants.bottomShooterPercent),
                    Math.abs(bottomShooterSubsystem.velocity.in(Value)),
                    Constants.shooterTolerancePercent);
            boolean condition = topCondition && bottomCondition;
            SmartDashboard.putBoolean("ShooterAt Speed", condition);
            return condition;
        };

        private static final BooleanSupplier elevatorAndTiltAtPositionCondition = () -> {
            boolean condition = BasicCommands.Elevator.createAtPositionCondition.apply(Constants.elevatorPosition)
                    .getAsBoolean() &&
                    BasicCommands.Tilt.createAtAngleCondition.apply(tiltAngle)
                            .getAsBoolean();
            SmartDashboard.putBoolean("Elevator and Tilt At Position", condition);
            return condition;
        };

        private static final BooleanSupplier alignedWithTarget = () -> {
            boolean condition = Constants.rotationPIDController.atSetpoint();
            SmartDashboard.putBoolean("Aligned with Target", condition);
            return condition;
        };

        private static final BooleanSupplier elevatorTiltShooterDriveAtPositionCondition = () -> {
            return elevatorAndTiltAtPositionCondition.getAsBoolean() && alignedWithTarget.getAsBoolean()
                    && shootersAtSpeed.getAsBoolean();
        };

        private static final Runnable rotateDrive = () -> {
            double degrees = telemetrySubsystem.poseEstimate.get().getRotation().getDegrees();
            degrees %= 360.0;
            degrees = degrees >= 180.0 ? degrees - 360 : degrees;
            degrees = degrees < -180.0 ? degrees + 360 : degrees;

            if (degrees > 0) {
                ChassisSpeeds rotateSpeeds = new ChassisSpeeds(0, 0, Constants.findTargetOmegaRadiansPerSecond);
                driveSubsystem.controlRobotChassisSpeeds.apply(new Translation2d()).accept(rotateSpeeds);
            } else {
                ChassisSpeeds rotateSpeeds = new ChassisSpeeds(0, 0, -Constants.findTargetOmegaRadiansPerSecond);
                driveSubsystem.controlRobotChassisSpeeds.apply(new Translation2d()).accept(rotateSpeeds);
            }
        };

        private static final Supplier<Command> rotateDriveUntilTargetFoundCommand = () -> {
            Command command = Commands.run(rotateDrive, driveSubsystem).until(hasTarget);
            command.setName("Rotate Until Target Found");
            return command;
        };

        private static final Supplier<Command> rotateDriveUntilTargetFoundSpinShootersCommand = () -> {
            Command command = Commands.race(
                    rotateDriveUntilTargetFoundCommand.get(),
                    BasicCommands.BottomShooter.createSpinCommand.apply(Constants.bottomShooterPercent),
                    BasicCommands.TopShooter.createSpinCommand.apply(Constants.topShooterPercent));
            command.setName("Rotate Until Found Target");
            return command;
        };

        private static final Runnable setDistanceAndAngle = () -> {
            var distance = telemetrySubsystem.priorityTargetDistance.get();
            var angle = telemetrySubsystem.priorityTargetRotation.get();
            if (distance.isPresent() && angle.isPresent()) {
                targetDistance.mut_setMagnitude(distance.get());
                var aimAngle = Constants.distanceToAngle.apply(targetDistance);
                tiltAngle.mut_setMagnitude(aimAngle.in(Degrees));
                Constants.rotationPIDController.setSetpoint(angle.get().getDegrees());
                Constants.rotationPIDController.setTolerance(Constants.tolerance);
            }
        };

        private static final Supplier<Command> setDistanceAndAngleCommand = () -> {
            return Commands.runOnce(setDistanceAndAngle);
        };

        private static final Runnable rotateDriveToFoundTarget = () -> {
            var angle = telemetrySubsystem.priorityTargetRotation.get();
            if (angle.isPresent()) {
                double omegaRadiansPerSecond = Constants.rotationPIDController.calculate(angle.get().getDegrees());
                ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, omegaRadiansPerSecond);
                driveSubsystem.controlRobotChassisSpeeds.apply(new Translation2d()).accept(chassisSpeeds);
            }
        };

        private static final Supplier<Command> rotateDriveToAlignTargetCommand = () -> {
            Command command = Commands.run(rotateDriveToFoundTarget, driveSubsystem);
            command.setName("Rotate To Align Target");
            return command;
        };

        private static final Supplier<Command> createAimCommand = () -> {
            Command command = Commands.parallel(
                    rotateDriveToAlignTargetCommand.get(),
                    BasicCommands.Elevator.createSetAndHoldElevatorPositionCommand
                            .apply(Constants.elevatorPosition),
                    BasicCommands.Tilt.createSetAndHoldTiltAngleCommand.apply(tiltAngle),
                    BasicCommands.TopShooter.createSpinCommand.apply(Constants.topShooterPercent),
                    BasicCommands.BottomShooter.createSpinCommand
                            .apply(Constants.bottomShooterPercent))
                    .until(elevatorTiltShooterDriveAtPositionCondition);
            command.setName("Ranged Aim");
            return command;
        };

        private static final Supplier<Command> createShootCommand = () -> {
            Command command = Commands.parallel(
                    Commands.run(driveSubsystem.stop, driveSubsystem),
                    BasicCommands.Elevator.createSetAndHoldElevatorPositionCommand
                            .apply(Constants.elevatorPosition),
                    BasicCommands.Tilt.createSetAndHoldTiltAngleCommand.apply(tiltAngle),
                    BasicCommands.TopShooter.createSpinCommand.apply(Constants.topShooterPercent),
                    BasicCommands.BottomShooter.createSpinCommand
                            .apply(Constants.bottomShooterPercent),
                    BasicCommands.Singulator.createSpinCommand.apply(Constants.singulatorPercent),
                    BasicCommands.Transport.createSpinCommand.apply(Constants.transportPercent));
            command.setName("Ranged Shoot");
            return command;
        };

        public static final Supplier<Command> create = () -> {
            return Commands.sequence(
                    rotateDriveUntilTargetFoundSpinShootersCommand.get(),
                    setDistanceAndAngleCommand.get(),
                    rotateDriveUntilTargetFoundSpinShootersCommand.get(),
                    createAimCommand.get(),
                    createShootCommand.get());
        };

        public static final Supplier<Command> createWithDelay = () -> {
            return Commands.sequence(
                    rotateDriveUntilTargetFoundSpinShootersCommand.get(),
                    setDistanceAndAngleCommand.get(),
                    rotateDriveUntilTargetFoundSpinShootersCommand.get(),
                    createAimCommand.get(),
                    createShootCommand.get()
                            .raceWith(Commands.waitSeconds(Constants.endOfShootDelay)));
        };
    }
}
