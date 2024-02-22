// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.function.BooleanSupplier;

import com.controllers.Controller;
import com.ctre.phoenix6.Orchestra;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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
import frc.robot.subsystems.LEDSubsystem;
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
                                new PIDConstants(5),
                                new PIDConstants(5),
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
        public static final LEDSubsystem ledSubsystem = LEDSubsystem.create.get();

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

                operator.back.onTrue(ManualCommands.ResetAndHolding.level);
                operator.leftY.whileTrue(ManualCommands.Elevator.drive);
                operator.rightY.whileTrue(ManualCommands.Tilt.drive);
                operator.fullTrigger.whileTrue(ManualCommands.Shooter.command);
                operator.b.whileTrue(ManualCommands.Pickup.barf);
                operator.y.whileTrue(ManualCommands.Scoring.spinUpCommand);
                operator.x.whileTrue(ManualCommands.Scoring.climbPrep);
                operator.x.onFalse(ManualCommands.Scoring.climbEnd);
                operator.a.whileTrue(ManualCommands.Scoring.ampPosition);
                operator.a.onFalse(ManualCommands.Scoring.ampPositionReset);
                operator.padDown.whileTrue(ScoringCommands.createTest.get());
                operator.leftBumper.whileTrue(ManualCommands.Pickup.groundPickup);
                operator.leftBumper.onFalse(ManualCommands.Pickup.groundPickupReset);
                operator.rightBumper.whileTrue(ManualCommands.Pickup.babyBirdPickup);
                operator.rightBumper.onFalse(ManualCommands.Pickup.babyBirdPickupReset);

                driver.leftBumper.whileTrue(ManualCommands.Scoring.fenderScore);
                driver.leftBumper.onFalse(ManualCommands.Scoring.fenderReset);
                driver.back.whileTrue(DriveCommands.createZeroModulesCommand.get());

                driver.rightBumper.whileTrue(ManualCommands.Scoring.rangedScore);
                driver.start.onTrue(TelemetryCommands.createSetStartPoseCommand
                                .apply(TelemetryCommands.Constants.SpeakerTopStart));
                driver.y.onTrue(DriveCommands.createResetEncodersCommand.get());
                driver.x.whileTrue(PickupCommands.createNoteDetect.get());

                
                
                driver.a.whileTrue(ManualCommands.Scoring.ampDrop);
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
                // Runnable loadRhapsody = () -> orchestra.loadMusic("rhapsody.chrp");
                // Runnable loadGiveYouUp = () ->
                // orchestra.loadMusic("Never-Gonna-Give-You-Up-1.chrp");
                // Runnable loadHogan = () -> orchestra.loadMusic("HogansHeroes.chrp");
                // Runnable loadEastBound = () -> orchestra.loadMusic("EastBoundAndDown.chrp");

                // Runnable playOrchestra = () -> {
                // orchestra.play();
                // };
                // Command playRhapsodyCommand =
                // Commands.runOnce(loadRhapsody).andThen(Commands.runOnce(playOrchestra));
                // Command playGiveYouUp =
                // Commands.runOnce(loadGiveYouUp).andThen(Commands.runOnce(playOrchestra));
                // Command playHogan =
                // Commands.runOnce(loadHogan).andThen(Commands.runOnce(playOrchestra));
                // Command playEastBound =
                // Commands.runOnce(loadEastBound).andThen(Commands.runOnce(playOrchestra));

                // Runnable stopOrchestra = () -> {
                // orchestra.stop();
                // };
                // Command stopRhapsody = Commands.runOnce(stopOrchestra);
                // Command stopGiveYouUp = Commands.runOnce(stopOrchestra);
                // Command stopHogan = Commands.runOnce(stopOrchestra);
                // Command stopEastBound = Commands.runOnce(stopOrchestra);

                // driver.a.whileTrue(playRhapsodyCommand);
                // driver.a.onFalse(stopRhapsody);
                // driver.b.whileTrue(playGiveYouUp);
                // driver.b.onFalse(stopGiveYouUp);
                // driver.x.whileTrue(playHogan);
                // driver.x.onFalse(stopHogan);
                // driver.y.whileTrue(playEastBound);
                // driver.y.onFalse(stopEastBound);

                new AutoCommands();

                autoChooser = AutoBuilder.buildAutoChooser();
                SmartDashboard.putData("Auto Chooser", autoChooser);

        }

        public void resetEncoders() {
                shooterTiltSubsystem.resetRelEncoderFromAbsolute.run();
                driveSubsystem.resetSteerEncodersFromAbsolutes.run();
        }

        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }
}
