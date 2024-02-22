package frc.robot.commandCreators;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics.SwerveDriveWheelStates;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

public class DriveCommands {

        private static final class Constants {
                private static final class NoteDetection {
                        private static final PathConstraints pathConstraints = new PathConstraints(
                                        DriveSubsystem.Constants.maxLinearVelocity.in(MetersPerSecond),
                                        DriveSubsystem.Constants.maxLinearVelocity.in(MetersPerSecond),
                                        DriveSubsystem.Constants.maxAngularVelocity.in(RadiansPerSecond),
                                        DriveSubsystem.Constants.maxAngularVelocity.in(RadiansPerSecond));
                        private static final double goalEndVelocityMPS = 0.0;
                        private static final double rotationDelayDistance = 0.0;
                }

                private static final class Source {
                        private static final Pose2d source = new Pose2d(
                                        15.438237,
                                        0.962824,
                                        Rotation2d.fromDegrees(-60.0));
                }
        }

        public static final Supplier<Command> createResetEncodersCommand = () -> {
                Runnable reset = () -> RobotContainer.driveSubsystem.resetSteerEncodersFromAbsolutes.run();

                Command command = Commands.runOnce(reset, RobotContainer.driveSubsystem);
                command.setName("Reset Modules");
                return command;
        };

        public static final Supplier<Command> createZeroModulesCommand = () -> {
                SwerveDriveWheelStates zeroStates = new SwerveDriveWheelStates(
                                new SwerveModuleState[] {
                                                new SwerveModuleState(),
                                                new SwerveModuleState(),
                                                new SwerveModuleState(),
                                                new SwerveModuleState()
                                });
                Runnable zero = () -> RobotContainer.driveSubsystem.controlModules.accept(zeroStates);

                Command command = Commands.run(zero, RobotContainer.driveSubsystem);
                command.setName("Zero Modules");
                return command;
        };

        public static final Supplier<Command> createTestFieldCentricCommand = () -> {
                Translation2d cor = new Translation2d();
                Runnable drive = () -> {
                        ChassisSpeeds speeds = new ChassisSpeeds(1, 0, 0);
                        ChassisSpeeds adjustedSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds,
                                        RobotContainer.telemetrySubsystem.poseEstimate.get().getRotation());
                        speeds.vxMetersPerSecond = adjustedSpeeds.vxMetersPerSecond;
                        speeds.vyMetersPerSecond = adjustedSpeeds.vyMetersPerSecond;
                        speeds.omegaRadiansPerSecond = adjustedSpeeds.omegaRadiansPerSecond;
                        RobotContainer.driveSubsystem.controlRobotChassisSpeeds
                                        .apply(cor)
                                        .accept(speeds);
                };
                Command command = Commands.run(drive, RobotContainer.driveSubsystem);
                command.setName("Test Field Centric");
                return command;
        };

        public static final Supplier<Command> createRobotCentricCommand = () -> {
                ChassisSpeeds speeds = new ChassisSpeeds();
                Translation2d cor = new Translation2d();
                Runnable drive = () -> {
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
                Command command = Commands.run(drive, RobotContainer.driveSubsystem);
                command.setName("Manual Robot Centric");
                return command;
        };

        public static final Supplier<Command> createRobotCentricFLCommand = () -> {
                ChassisSpeeds speeds = new ChassisSpeeds();
                Translation2d cor = new Translation2d(
                                Units.inchesToMeters(13),
                                Units.inchesToMeters(13));
                Runnable drive = () -> {
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
                Command command = Commands.run(drive, RobotContainer.driveSubsystem);
                command.setName("Manual Robot Centric");
                return command;
        };

        public static final Supplier<Command> createRobotCentricFRCommand = () -> {
                ChassisSpeeds speeds = new ChassisSpeeds();
                Translation2d cor = new Translation2d(
                                Units.inchesToMeters(13),
                                Units.inchesToMeters(-13));
                Runnable drive = () -> {
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
                Command command = Commands.run(drive, RobotContainer.driveSubsystem);
                command.setName("Manual Robot Centric");
                return command;
        };

        public static final Supplier<Command> createFieldCentricCommand = () -> {
                ChassisSpeeds speeds = new ChassisSpeeds();
                Translation2d cor = new Translation2d();
                Runnable drive = () -> {
                        speeds.vxMetersPerSecond = DriveSubsystem.Constants.maxLinearVelocity
                                        .in(MetersPerSecond) * RobotContainer.driver.leftYValue.getAsDouble();
                        speeds.vyMetersPerSecond = DriveSubsystem.Constants.maxLinearVelocity
                                        .in(MetersPerSecond) * RobotContainer.driver.leftXValue.getAsDouble();
                        speeds.omegaRadiansPerSecond = DriveSubsystem.Constants.maxAngularVelocity
                                        .in(RadiansPerSecond)
                                        * RobotContainer.driver.fullTriggerValue.getAsDouble();
                        ChassisSpeeds adjustedSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds,
                                        RobotContainer.telemetrySubsystem.poseEstimate.get().getRotation());
                        speeds.vxMetersPerSecond = adjustedSpeeds.vxMetersPerSecond;
                        speeds.vyMetersPerSecond = adjustedSpeeds.vyMetersPerSecond;
                        speeds.omegaRadiansPerSecond = adjustedSpeeds.omegaRadiansPerSecond;
                        RobotContainer.driveSubsystem.controlRobotChassisSpeeds
                                        .apply(cor)
                                        .accept(speeds);
                };
                Command command = Commands.run(drive, RobotContainer.driveSubsystem);
                command.setName("Manual Field Centric");
                return command;
        };

        public static final Supplier<Command> createFieldCentricFLCommand = () -> {
                ChassisSpeeds speeds = new ChassisSpeeds();
                Translation2d cor = new Translation2d(
                                Units.inchesToMeters(13),
                                Units.inchesToMeters(13));
                Runnable drive = () -> {
                        speeds.vxMetersPerSecond = DriveSubsystem.Constants.maxLinearVelocity
                                        .in(MetersPerSecond) * RobotContainer.driver.leftYValue.getAsDouble();
                        speeds.vyMetersPerSecond = DriveSubsystem.Constants.maxLinearVelocity
                                        .in(MetersPerSecond) * RobotContainer.driver.leftXValue.getAsDouble();
                        speeds.omegaRadiansPerSecond = DriveSubsystem.Constants.maxAngularVelocity
                                        .in(RadiansPerSecond)
                                        * RobotContainer.driver.fullTriggerValue.getAsDouble();
                        ChassisSpeeds adjustedSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds,
                                        RobotContainer.telemetrySubsystem.poseEstimate.get().getRotation());
                        speeds.vxMetersPerSecond = adjustedSpeeds.vxMetersPerSecond;
                        speeds.vyMetersPerSecond = adjustedSpeeds.vyMetersPerSecond;
                        speeds.omegaRadiansPerSecond = adjustedSpeeds.omegaRadiansPerSecond;
                        RobotContainer.driveSubsystem.controlRobotChassisSpeeds
                                        .apply(cor)
                                        .accept(speeds);
                };
                Command command = Commands.run(drive, RobotContainer.driveSubsystem);
                command.setName("Manual Field Centric FL");
                return command;
        };
        public static final Supplier<Command> createFieldCentricFRCommand = () -> {
                ChassisSpeeds speeds = new ChassisSpeeds();
                Translation2d cor = new Translation2d(
                                Units.inchesToMeters(13),
                                Units.inchesToMeters(-13));
                Runnable drive = () -> {
                        speeds.vxMetersPerSecond = DriveSubsystem.Constants.maxLinearVelocity
                                        .in(MetersPerSecond) * RobotContainer.driver.leftYValue.getAsDouble();
                        speeds.vyMetersPerSecond = DriveSubsystem.Constants.maxLinearVelocity
                                        .in(MetersPerSecond) * RobotContainer.driver.leftXValue.getAsDouble();
                        speeds.omegaRadiansPerSecond = DriveSubsystem.Constants.maxAngularVelocity
                                        .in(RadiansPerSecond)
                                        * RobotContainer.driver.fullTriggerValue.getAsDouble();
                        ChassisSpeeds adjustedSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds,
                                        RobotContainer.telemetrySubsystem.poseEstimate.get().getRotation());
                        speeds.vxMetersPerSecond = adjustedSpeeds.vxMetersPerSecond;
                        speeds.vyMetersPerSecond = adjustedSpeeds.vyMetersPerSecond;
                        speeds.omegaRadiansPerSecond = adjustedSpeeds.omegaRadiansPerSecond;
                        RobotContainer.driveSubsystem.controlRobotChassisSpeeds
                                        .apply(cor)
                                        .accept(speeds);
                };
                Command command = Commands.run(drive, RobotContainer.driveSubsystem);
                command.setName("Manual Field Centric FR");
                return command;
        };

        public static final Supplier<Command> goToNoteCommandSupplier = () -> {
                        var pose = RobotContainer.telemetrySubsystem.objectPositions.get(0).getSecond().get();
                        if (pose.isPresent()) {
                                return AutoBuilder
                                                .pathfindToPose(
                                                                pose.get(),
                                                                Constants.NoteDetection.pathConstraints,
                                                                Constants.NoteDetection.goalEndVelocityMPS,
                                                                Constants.NoteDetection.rotationDelayDistance);
                        } else {
                                return Commands.none();
                        }
        };

        public static final Supplier<Command> createGoToSource = () -> {
                return AutoBuilder.pathfindToPoseFlipped(
                                Constants.Source.source,
                                Constants.NoteDetection.pathConstraints,
                                Constants.NoteDetection.goalEndVelocityMPS,
                                Constants.NoteDetection.rotationDelayDistance);

        };

        public DriveCommands() {

        }
}
