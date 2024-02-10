// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;
import java.util.Random;
import java.util.function.Supplier;

import com.controllers.Controller;
import com.ctre.phoenix6.Orchestra;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.CommandBinder;
import frc.robot.commands.CommandCreators;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PickupSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterTiltSubsystem;

public class RobotContainer {

        private static final class Constants {
                private static final class Controller {
                        private static final double deadband = 0.1;
                        private static final int driverPort = 0;
                        private static final int operatorPort = 1;
                }
        }

        public static final Orchestra orchestra = new Orchestra();

        private final SendableChooser<Command> autoChooser;

        private final DriveSubsystem driveSubsystem;

        public RobotContainer() {
                Controller driver = Controller.createFromXBox
                                .apply(Constants.Controller.driverPort)
                                .apply(Constants.Controller.deadband);
                Controller operator = Controller.createFromXBox
                                .apply(Constants.Controller.operatorPort)
                                .apply(Constants.Controller.deadband);

                PickupSubsystem pickupSubsystem = PickupSubsystem.create.get();
                ShooterSubsystem shooterSubsystem = ShooterSubsystem.create.get();
                ShooterTiltSubsystem shooterTiltSubsystem = ShooterTiltSubsystem.create.get();
                ElevatorSubsystem elevatorSubsystem = ElevatorSubsystem.create.get();
                driveSubsystem = DriveSubsystem.create.get();

                CommandBinder.bindManualPickupCommand
                                .apply(pickupSubsystem)
                                .apply(shooterSubsystem)
                                .apply(shooterTiltSubsystem)
                                .accept(operator.a);
                CommandBinder.bindManualFenderShootCommand
                                .apply(pickupSubsystem)
                                .apply(shooterSubsystem)
                                .apply(shooterTiltSubsystem)
                                .accept(operator.b);
                CommandBinder.bindManualShooterTiltCommand
                                .apply(shooterTiltSubsystem)
                                .apply(operator.rightYValue)
                                .accept(operator.rightY);
                CommandBinder.bindManualElevatorCommand
                                .apply(elevatorSubsystem)
                                .apply(operator.leftYValue)
                                .accept(operator.leftY);
                CommandBinder.bindManualResetSteerEncodersCommand
                                .apply(driveSubsystem)
                                .accept(driver.start);
                CommandBinder.bindManualModuleZeroCommand
                                .apply(driveSubsystem)
                                .accept(driver.back);
                CommandBinder.bindManualRobotCentricDriveCommand
                                .apply(driveSubsystem)
                                .accept(driver);
                Command fieldCentricCommand = CommandBinder.bindManualFieldCentricDriveCommand
                                .apply(driveSubsystem)
                                .apply(driver);
                CommandBinder.bindPathFindToPoseCommandToTrigger
                                .apply(driveSubsystem)
                                .apply(new Pose2d(4, 1, Rotation2d.fromDegrees(180)))
                                .apply(new PathConstraints(
                                                3.0, 4.0,
                                                Units.degreesToRadians(540), Units.degreesToRadians(720)))
                                .apply(0.0)
                                .apply(0.0)
                                .accept(driver.a);

                Random randX = new Random();
                Random randY = new Random();
                Random randTheta = new Random();

                Supplier<Optional<Pose2d>> targetPoseOptionalTest = () -> {
                        double x = 15 * randX.nextDouble();
                        double y = 7.5 * randY.nextDouble();
                        double thetaDegrees = 360 * randTheta.nextDouble();
                        Rotation2d theta = Rotation2d.fromDegrees(thetaDegrees);

                        return Optional.of(new Pose2d(
                                        x,
                                        y,
                                        theta));
                };

                CommandBinder.bindPathFindToSuppliedPoseCommandToTrigger
                                .apply(driveSubsystem)
                                .apply(targetPoseOptionalTest)
                                .apply(new PathConstraints(
                                                3.0, 4.0,
                                                Units.degreesToRadians(540), Units.degreesToRadians(720)))
                                .apply(0.0)
                                .apply(0.0)
                                .apply(false)
                                .apply(fieldCentricCommand)
                                .accept(driver.b);

                Supplier<Optional<Pose2d>> goToNoteSupplier = () -> {
                        var optionalPose = driveSubsystem.objectPositions.get(0).getSecond().get();
                        if (optionalPose.isEmpty()) {
                                return Optional.empty();
                        } else {
                                return Optional.of(new Pose2d(
                                                optionalPose.get().getTranslation(),
                                                optionalPose.get().getRotation()
                                                                .rotateBy(Rotation2d.fromDegrees(180))));
                        }
                };

                CommandBinder.bindPathFindToSuppliedPoseCommandToTrigger
                                .apply(driveSubsystem)
                                .apply(goToNoteSupplier)
                                .apply(new PathConstraints(
                                                3.0, 4.0,
                                                Units.degreesToRadians(540), Units.degreesToRadians(720)))
                                .apply(0.0)
                                .apply(0.0)
                                .apply(false)
                                .apply(fieldCentricCommand)
                                .accept(driver.x);

                SmartDashboard.putData("Operator", operator);
                SmartDashboard.putData("Pickup", pickupSubsystem);
                SmartDashboard.putData("Shooter", shooterSubsystem);
                SmartDashboard.putData("Shooter Tilt", shooterTiltSubsystem);
                SmartDashboard.putData("Elevator", elevatorSubsystem);
                SmartDashboard.putData("Drive", driveSubsystem);

                // Logging callback for the active path, this is sent as a list of poses
                PathPlannerLogging.setLogActivePathCallback((poses) -> {
                        driveSubsystem.field2d.getObject("path").setPoses(poses);
                });

                // Logging callback for target robot pose
                PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
                        // Do whatever you want with the pose here
                        driveSubsystem.field2d.getObject("target pose").setPose(pose);
                });

                NamedCommands.registerCommand(
                                "TestGoToNote",
                                CommandCreators.createSetPathFindCommand
                                                .apply(driveSubsystem)
                                                .apply(targetPoseOptionalTest)
                                                .apply(new PathConstraints(
                                                                3.0, 4.0,
                                                                Units.degreesToRadians(540),
                                                                Units.degreesToRadians(720)))
                                                .apply(0.0)
                                                .apply(0.0)
                                                .apply(true)
                                                .andThen(CommandCreators.pathFindToSuppliedOptPoseCommand[0]));
                Runnable playOrchestra = () -> {
                        orchestra.play();
                };
                Command playOrchestraCommand = Commands.runOnce(playOrchestra);

                Runnable stopOrchestra = () -> {
                        orchestra.stop();
                };
                Command stopOrchestraCommand = Commands.runOnce(stopOrchestra);

                driver.y.whileTrue(playOrchestraCommand);
                driver.y.onFalse(stopOrchestraCommand);


                
                autoChooser = AutoBuilder.buildAutoChooser();

                SmartDashboard.putData("Auto Chooser", autoChooser);

        }

        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }
}
