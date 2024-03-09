package frc.robot.commandCreators;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.RobotContainer.driveSubsystem;

import java.util.function.Function;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics.SwerveDriveWheelStates;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

public class DriveCommands {

        private static final class DriveConstants {
                private static final Translation2d originCOR = new Translation2d();
                private static final Translation2d bottomLeftCOR = new Translation2d(
                                Units.inchesToMeters(-20),
                                Units.inchesToMeters(20));
                private static final Translation2d bottomRightCOR = new Translation2d(
                                Units.inchesToMeters(-20),
                                Units.inchesToMeters(-20));
        }

        private static final Function<Translation2d, Runnable> createRobotCentricDrive = (cor) -> {
                ChassisSpeeds speeds = new ChassisSpeeds();
                return () -> {
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
        };

        private static final Function<Translation2d, Runnable> createFieldCentricDrive = (cor) -> {
                ChassisSpeeds speeds = new ChassisSpeeds();
                return () -> {
                        double flipper = 1;
                        var color = DriverStation.getAlliance();
                        if (color.isPresent() && color.get() == Alliance.Red) {
                                flipper *= -1;
                        }
                        speeds.vxMetersPerSecond = DriveSubsystem.Constants.maxLinearVelocity
                                        .in(MetersPerSecond) * RobotContainer.driver.leftYValue.getAsDouble();
                        speeds.vyMetersPerSecond = DriveSubsystem.Constants.maxLinearVelocity
                                        .in(MetersPerSecond) * RobotContainer.driver.leftXValue.getAsDouble();
                        speeds.omegaRadiansPerSecond = DriveSubsystem.Constants.maxAngularVelocity
                                        .in(RadiansPerSecond)
                                        * RobotContainer.driver.fullTriggerValue.getAsDouble();
                        speeds.vxMetersPerSecond *= flipper;
                        speeds.vyMetersPerSecond *= flipper;
                        ChassisSpeeds adjustedSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds,
                                        RobotContainer.telemetrySubsystem.poseEstimate.get().getRotation());
                        speeds.vxMetersPerSecond = adjustedSpeeds.vxMetersPerSecond;
                        speeds.vyMetersPerSecond = adjustedSpeeds.vyMetersPerSecond;
                        speeds.omegaRadiansPerSecond = adjustedSpeeds.omegaRadiansPerSecond;
                        RobotContainer.driveSubsystem.controlRobotChassisSpeeds
                                        .apply(cor);
                };
        };

        public static final Supplier<Command> createFieldCentricDriveOriginCommand = () -> {
                Command command = Commands.run(
                                createFieldCentricDrive.apply(DriveConstants.originCOR),
                                driveSubsystem);
                command.setName("Manual FC Origin Drive");
                return command;
        };

        public static final Supplier<Command> createFieldCentricDriveRLCommand = () -> {
                Command command = Commands.run(
                                createFieldCentricDrive.apply(DriveConstants.bottomLeftCOR),
                                driveSubsystem);
                command.setName("Manual FC Rear Left Drive");
                return command;
        };

        public static final Supplier<Command> createFieldCentricDriveRRCommand = () -> {
                Command command = Commands.run(
                                createFieldCentricDrive.apply(DriveConstants.bottomRightCOR),
                                driveSubsystem);
                command.setName("Manual FC Rear Right Drive");
                return command;
        };

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
                                Units.inchesToMeters(-20),
                                Units.inchesToMeters(20));
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
                                Units.inchesToMeters(-20),
                                Units.inchesToMeters(-20));
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
                        double flipper = 1;
                        var color = DriverStation.getAlliance();
                        if (color.isPresent() && color.get() == Alliance.Red) {
                                flipper *= -1;
                        }
                        speeds.vxMetersPerSecond = DriveSubsystem.Constants.maxLinearVelocity
                                        .in(MetersPerSecond) * RobotContainer.driver.leftYValue.getAsDouble();
                        speeds.vyMetersPerSecond = DriveSubsystem.Constants.maxLinearVelocity
                                        .in(MetersPerSecond) * RobotContainer.driver.leftXValue.getAsDouble();
                        speeds.omegaRadiansPerSecond = DriveSubsystem.Constants.maxAngularVelocity
                                        .in(RadiansPerSecond)
                                        * RobotContainer.driver.fullTriggerValue.getAsDouble();
                        speeds.vxMetersPerSecond *= flipper;
                        speeds.vyMetersPerSecond *= flipper;
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
                                Units.inchesToMeters(-20),
                                Units.inchesToMeters(20));
                Runnable drive = () -> {
                        double flipper = 1;
                        var color = DriverStation.getAlliance();
                        if (color.isPresent() && color.get() == Alliance.Red) {
                                flipper *= -1;
                        }
                        speeds.vxMetersPerSecond = DriveSubsystem.Constants.maxLinearVelocity
                                        .in(MetersPerSecond) * RobotContainer.driver.leftYValue.getAsDouble();
                        speeds.vyMetersPerSecond = DriveSubsystem.Constants.maxLinearVelocity
                                        .in(MetersPerSecond) * RobotContainer.driver.leftXValue.getAsDouble();
                        speeds.omegaRadiansPerSecond = DriveSubsystem.Constants.maxAngularVelocity
                                        .in(RadiansPerSecond)
                                        * RobotContainer.driver.fullTriggerValue.getAsDouble();
                        speeds.vxMetersPerSecond *= flipper;
                        speeds.vyMetersPerSecond *= flipper;
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
                                Units.inchesToMeters(-20),
                                Units.inchesToMeters(-20));
                Runnable drive = () -> {
                        double flipper = 1;
                        var color = DriverStation.getAlliance();
                        if (color.isPresent() && color.get() == Alliance.Red) {
                                flipper *= -1;
                        }
                        speeds.vxMetersPerSecond = DriveSubsystem.Constants.maxLinearVelocity
                                        .in(MetersPerSecond) * RobotContainer.driver.leftYValue.getAsDouble();
                        speeds.vyMetersPerSecond = DriveSubsystem.Constants.maxLinearVelocity
                                        .in(MetersPerSecond) * RobotContainer.driver.leftXValue.getAsDouble();
                        speeds.omegaRadiansPerSecond = DriveSubsystem.Constants.maxAngularVelocity
                                        .in(RadiansPerSecond)
                                        * RobotContainer.driver.fullTriggerValue.getAsDouble();
                        speeds.vxMetersPerSecond *= flipper;
                        speeds.vyMetersPerSecond *= flipper;
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

        public DriveCommands() {

        }
}
