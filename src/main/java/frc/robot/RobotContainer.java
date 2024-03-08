// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.Optional;
import java.util.function.BooleanSupplier;

import com.controllers.Controller;
import com.ctre.phoenix6.Orchestra;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commandCreators.BasicCommands;
import frc.robot.commandCreators.DriveCommands;
import frc.robot.commandCreators.PickupCommands;
import frc.robot.commandCreators.ScoringCommands;
import frc.robot.commandCreators.TelemetryCommands;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.ManualCommands;
import frc.robot.subsystems.BottomShooterSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.InnerIntakeSubsystem;
import frc.robot.subsystems.MechanismSubsystem;
import frc.robot.subsystems.NotedLoadedSubsystem;
import frc.robot.subsystems.OuterIntakeSubsystem;
import frc.robot.subsystems.TopShooterSubsystem;
import frc.robot.subsystems.TransportSubsystem;
import frc.robot.triggers.ComplexTriggers;
import frc.robot.subsystems.ShooterTiltSubsystem;
import frc.robot.subsystems.SingulatorSubsystem;
import frc.robot.subsystems.TelemetrySubsystem;

public class RobotContainer {

        private static final class Constants {
                private static final class Controller {
                        private static final double deadband = 0.1;
                        private static final int driverPort = 0;
                        private static final int operatorPort = 1;
                        // private static final int testPort = 2;
                }

                private static final ReplanningConfig replanningConfig = new ReplanningConfig(
                                true,
                                true,
                                0.5,
                                0.5);

                private static final HolonomicPathFollowerConfig config = new HolonomicPathFollowerConfig(
                                new PIDConstants(2),
                                new PIDConstants(2),
                                DriveSubsystem.Constants.maxLinearVelocity.in(MetersPerSecond),
                                DriveSubsystem.Constants.driveRadius.in(Meters),
                                replanningConfig,
                                0.02);

                private static final BooleanSupplier shouldFlipPath = () -> {
                        var alliance = DriverStation.getAlliance();
                        if (alliance.isPresent()) {
                                return alliance.get() == DriverStation.Alliance.Red;
                        }
                        return false;
                };
        }

        public static final Orchestra orchestra = new Orchestra();
        public static final Controller driver = Controller.createFromXBox
                        .apply(Constants.Controller.driverPort)
                        .apply(Constants.Controller.deadband);
        public static final Controller operator = Controller.createFromXBox
                        .apply(Constants.Controller.operatorPort)
                        .apply(Constants.Controller.deadband);
        // public static final Controller test = Controller.createFromXBox
        // .apply(Constants.Controller.testPort)
        // .apply(Constants.Controller.deadband);

        public static final TopShooterSubsystem topShooterSubsystem = TopShooterSubsystem.create.get();
        public static final BottomShooterSubsystem bottomShooterSubsystem = BottomShooterSubsystem.create.get();
        public static final OuterIntakeSubsystem outerIntakeSubsystem = OuterIntakeSubsystem.create.get();
        public static final InnerIntakeSubsystem innerIntakeSubsystem = InnerIntakeSubsystem.create.get();
        public static final TransportSubsystem transportSubsystem = TransportSubsystem.create.get();
        public static final SingulatorSubsystem singulatorSubsystem = SingulatorSubsystem.create.get();
        public static final NotedLoadedSubsystem notedLoadedSubsystem = NotedLoadedSubsystem.create.get();
        public static final ShooterTiltSubsystem shooterTiltSubsystem = ShooterTiltSubsystem.create.get();
        public static final ElevatorSubsystem elevatorSubsystem = ElevatorSubsystem.create.get();
        public static final DriveSubsystem driveSubsystem = DriveSubsystem.create.get();
        public static final TelemetrySubsystem telemetrySubsystem = TelemetrySubsystem.create.get();
        public static final MechanismSubsystem mechanismSubsystem = new MechanismSubsystem();
        // public static final LEDSubsystem ledSubsystem = LEDSubsystem.create.get();

        private final SendableChooser<Command> autoChooser;

        public RobotContainer() {
                AutoBuilder.configureHolonomic(
                                telemetrySubsystem.poseEstimate,
                                telemetrySubsystem.setPoseEstimator,
                                driveSubsystem.robotSpeeds,
                                driveSubsystem.controlRobotChassisSpeeds.apply(new Translation2d()),
                                Constants.config,
                                Constants.shouldFlipPath,
                                driveSubsystem);
                SmartDashboard.putData("Outer Intake", outerIntakeSubsystem);
                SmartDashboard.putData("Inner Intake", innerIntakeSubsystem);
                SmartDashboard.putData("Transport", transportSubsystem);
                SmartDashboard.putData("Singulator", singulatorSubsystem);
                SmartDashboard.putData("NoteLoaded", notedLoadedSubsystem);
                SmartDashboard.putData("Top Shooter", topShooterSubsystem);
                SmartDashboard.putData("Bottom Shooter", bottomShooterSubsystem);
                SmartDashboard.putData("Shooter Tilt", shooterTiltSubsystem);
                SmartDashboard.putData("Elevator", elevatorSubsystem);
                SmartDashboard.putData("Drive", driveSubsystem);
                SmartDashboard.putData("Telemetry", telemetrySubsystem);

                operator.back.onTrue(PickupCommands.Level.command.get());
                operator.leftY.whileTrue(BasicCommands.Elevator.createDriveElevatorCommand.apply(operator.leftYValue));
                operator.rightY.whileTrue(BasicCommands.Tilt.createRotateTiltCommand.apply(operator.rightYValue));
                operator.fullTrigger.whileTrue(BasicCommands.ManualSpinners.spinCommand.get());
                operator.b.whileTrue(PickupCommands.Barf.barfCommand.get());
                operator.y.whileTrue(ScoringCommands.SpinUp.createSpinUp.get());
                operator.x.whileTrue(ScoringCommands.Climb.createClimbPrep.get());
                operator.x.onFalse(ScoringCommands.Climb.createClimbEnd.get());
                operator.a.whileTrue(ScoringCommands.Amp.createAmpPosition.get());
                operator.leftBumper.whileTrue(PickupCommands.Ground.groundCommandThenLevel.get());
                operator.leftBumper.onFalse(PickupCommands.Level.command.get());
                operator.rightBumper.whileTrue(PickupCommands.BabyBird.babyBirdCommandThenLevel.get());
                operator.rightBumper.onFalse(PickupCommands.Level.command.get());

                driver.leftBumper.whileTrue(ScoringCommands.Fender.create.get());
                driver.leftBumper.onFalse(PickupCommands.Level.command.get());
                driver.back.whileTrue(DriveCommands.createZeroModulesCommand.get());

                driver.start.onTrue(TelemetryCommands.createSetStartPoseCommand
                                .apply(TelemetryCommands.Constants.testingStartPose));
                driver.x.whileTrue(PickupCommands.createNoteDetect.get());
                driver.x.onFalse(TelemetryCommands.createSetFrontCameraToFieldCommand.get());

                driver.a.whileTrue(ScoringCommands.Amp.createAmpDrop.get());
                driver.a.onFalse(PickupCommands.Level.command.get());
                driver.b.onTrue(TelemetryCommands.createResetPoseFromFrontCameraCommand.get());
                ComplexTriggers.robotCentricOriginDriveTrigger
                                .whileTrue(ManualCommands.Drive.RobotCentric.command);
                ComplexTriggers.fieldCentricOriginDriveTrigger
                                .whileTrue(ManualCommands.Drive.FieldCentric.command);
                ComplexTriggers.fieldCentricFLDriveTrigger.whileTrue(ManualCommands.Drive.FieldCentric.frontLeft);
                ComplexTriggers.fieldCentricFLDriveTrigger.onFalse(DriveCommands.createFieldCentricCommand.get());
                ComplexTriggers.fieldCentricFRDriveTrigger.whileTrue(ManualCommands.Drive.FieldCentric.frontRight);
                ComplexTriggers.fieldCentricFRDriveTrigger.onFalse(DriveCommands.createFieldCentricCommand.get());

                ComplexTriggers.robotCentricFLDriveTrigger.whileTrue(ManualCommands.Drive.RobotCentric.frontLeftRC);
                ComplexTriggers.robotCentricFLDriveTrigger.onFalse(DriveCommands.createRobotCentricCommand.get());
                ComplexTriggers.robotCentricFRDriveTrigger.whileTrue(ManualCommands.Drive.RobotCentric.frontRightRC);
                ComplexTriggers.robotCentricFRDriveTrigger.onFalse(DriveCommands.createRobotCentricCommand.get());

                PathPlannerLogging.setLogActivePathCallback((poses) -> {
                        // Do whatever you want with the poses here
                        telemetrySubsystem.field2d.getObject("path").setPoses(poses);
                });

                PPHolonomicDriveController.setRotationTargetOverride(this::getRotationTargetOverride);

                new AutoCommands();

                autoChooser = AutoBuilder.buildAutoChooser();
                SmartDashboard.putData("Auto Chooser", autoChooser);

        }

        public Optional<Rotation2d> getRotationTargetOverride() {
                // Some condition that should decide if we want to override rotation
                if (driver.x.getAsBoolean()) {
                        // Return an optional containing the rotation override (this should be a field
                        // relative rotation)
                        var poses = telemetrySubsystem.field2d.getObject("path").getPoses();
                        if (poses.size() != 0) {
                                var lastPose = poses.get(poses.size() - 1);
                                var secondLastPose = poses.get(poses.size() - 2);
                                var vector = lastPose.getTranslation().minus(secondLastPose.getTranslation());
                                return Optional.of(vector.getAngle());
                        } else {
                                return Optional.empty();
                        }

                } else {
                        // return an empty optional when we don't want to override the path's rotation
                        return Optional.empty();
                }
        }

        public void resetEncoders() {
                shooterTiltSubsystem.resetRelEncoderFromAbsolute.run();
                driveSubsystem.resetSteerEncodersFromAbsolutes.run();
        }

        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }
}
