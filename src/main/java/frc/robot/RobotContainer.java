// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.controllers.Controller;
import com.ctre.phoenix6.Orchestra;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.ManualCommands;
import frc.robot.subsystems.BottomShooterSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.InnerIntakeSubsystem;
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
                }
        }

        public static final Orchestra orchestra = new Orchestra();
        public static final Controller driver = Controller.createFromXBox
                        .apply(Constants.Controller.driverPort)
                        .apply(Constants.Controller.deadband);
        public static final Controller operator = Controller.createFromXBox
                        .apply(Constants.Controller.operatorPort)
                        .apply(Constants.Controller.deadband);

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
        // private final SendableChooser<Command> autoChooser;

        public RobotContainer() {
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
                ComplexTriggers.robotCentricOriginDriveTrigger
                                .whileTrue(ManualCommands.Drive.RobotCentric.command);
                ComplexTriggers.fieldCentricOriginDriveTrigger
                                .whileTrue(ManualCommands.Drive.FieldCentric.command);

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
        }

        public Command getAutonomousCommand() {
                // return autoChooser.getSelected();
                return Commands.none();
        }
}
