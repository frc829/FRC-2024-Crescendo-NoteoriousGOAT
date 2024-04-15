// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.controllers.Controller;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BottomShooterSubsystem;

public class RobotContainer {

        private static final class Constants {
                private static final class Controller {
                        private static final double deadband = 0.1;
                        private static final int driverPort = 0;
                        private static final int operatorPort = 1;
                }
        }

        public static final Controller driver = Controller.createFromXBox
                        .apply(Constants.Controller.driverPort)
                        .apply(Constants.Controller.deadband)
                        .apply(false);
        public static final Controller operator = Controller.createFromXBox
                        .apply(Constants.Controller.operatorPort)
                        .apply(Constants.Controller.deadband)
                        .apply(true);

        public static final BottomShooterSubsystem bottomShooterSubsystem = new BottomShooterSubsystem();

        public static final PowerDistribution pdh = new PowerDistribution();

        private final SendableChooser<Command> autoChooser;

        public RobotContainer() {
                autoChooser = new SendableChooser<>();
                bottomShooterSubsystem.setDefaultCommand(bottomShooterSubsystem.createStopCommand());
                operator.a.whileTrue(bottomShooterSubsystem.createSetVelocityRatioCommand(() -> 0.5));


                SmartDashboard.putData("PDH", pdh);
                SmartDashboard.putData("Bottom Shooter", bottomShooterSubsystem);
        }

        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }

        public void update(){
                bottomShooterSubsystem.update();
        }
}
