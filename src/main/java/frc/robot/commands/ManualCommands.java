package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics.SwerveDriveWheelStates;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.commandCreators.BasicCommands;
import frc.robot.commandCreators.PickupCommands;
import frc.robot.subsystems.DriveSubsystem;

public class ManualCommands {
        public static final class Elevator {
                public static final Command drive = BasicCommands.Set.ElevatorDrive.create
                                .apply(RobotContainer.operator.leftYValue);
        }

        public static final class Tilt {
                public static final Command drive = BasicCommands.Set.TiltDrive.create
                                .apply(RobotContainer.operator.rightYValue);
        }

        public static final class Shooter {
                public static final Runnable shooterSpin = () -> {
                        RobotContainer.topShooterSubsystem.spin
                                        .accept(RobotContainer.operator.fullTriggerValue.getAsDouble());
                        RobotContainer.bottomShooterSubsystem.spin
                                        .accept(RobotContainer.operator.fullTriggerValue.getAsDouble());
                };
                public static final Command command = Commands.run(shooterSpin,
                                RobotContainer.topShooterSubsystem,
                                RobotContainer.bottomShooterSubsystem);
        }

        public static final class Drive {
                public static final class ZeroModules {
                        private static final SwerveDriveWheelStates wheelStates = new SwerveDriveWheelStates(
                                        new SwerveModuleState[] {
                                                        new SwerveModuleState(),
                                                        new SwerveModuleState(),
                                                        new SwerveModuleState(),
                                                        new SwerveModuleState()
                                        });
                        private static final Runnable zero = () -> RobotContainer.driveSubsystem.controlModules
                                        .accept(wheelStates);
                        public static final Command command = Commands.run(zero, RobotContainer.driveSubsystem);
                }

                public static final class RobotCentric {
                        private static final ChassisSpeeds speeds = new ChassisSpeeds();
                        private static final Translation2d cor = new Translation2d();
                        private static final Runnable drive = () -> {
                                speeds.vxMetersPerSecond = DriveSubsystem.Constants.maxLinearVelocity
                                                .in(MetersPerSecond) * RobotContainer.driver.rightYValue.getAsDouble();
                                speeds.vyMetersPerSecond = DriveSubsystem.Constants.maxLinearVelocity
                                                .in(MetersPerSecond) * RobotContainer.driver.rightXValue.getAsDouble();
                                speeds.omegaRadiansPerSecond = DriveSubsystem.Constants.maxAngularVelocity
                                                .in(RadiansPerSecond)
                                                * RobotContainer.driver.fullTriggerValue.getAsDouble();
                                RobotContainer.driveSubsystem.controlRobotChassisSpeeds
                                                .apply(cor)
                                                .accept(speeds);
                        };
                        public static final Command command = Commands.run(drive, RobotContainer.driveSubsystem);
                }

                public static final class FieldCentric {
                        private static ChassisSpeeds speeds = new ChassisSpeeds();
                        private static final Translation2d cor = new Translation2d();
                        private static final Runnable drive = () -> {
                                speeds.vxMetersPerSecond = DriveSubsystem.Constants.maxLinearVelocity
                                                .in(MetersPerSecond) * RobotContainer.driver.leftYValue.getAsDouble();
                                speeds.vyMetersPerSecond = DriveSubsystem.Constants.maxLinearVelocity
                                                .in(MetersPerSecond) * RobotContainer.driver.leftXValue.getAsDouble();
                                speeds.omegaRadiansPerSecond = DriveSubsystem.Constants.maxAngularVelocity
                                                .in(RadiansPerSecond)
                                                * RobotContainer.driver.fullTriggerValue.getAsDouble();
                                speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds,
                                                RobotContainer.telemetrySubsystem.poseEstimate.get().getRotation());
                                RobotContainer.driveSubsystem.controlRobotChassisSpeeds
                                                .apply(cor)
                                                .accept(speeds);
                        };
                        public static final Command command = Commands.run(drive, RobotContainer.driveSubsystem);
                }
        }

        public static final class Pickup {
                public static final Command barf = PickupCommands.createBarf.get();
                public static final Command groundPickup = PickupCommands.createGround.get();
        }

        static {
                Elevator.drive.setName("Manual Elevator");
                Tilt.drive.setName("Manual Tilt");
                Shooter.command.setName("Manual Shooter");
                Drive.FieldCentric.command.setName("Manual Field Centric");
                Drive.RobotCentric.command.setName("Manual Robot Centric");
                Drive.ZeroModules.command.setName("Zero Modules");
        }
}
