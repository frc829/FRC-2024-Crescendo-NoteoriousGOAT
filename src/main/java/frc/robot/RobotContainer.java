// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.controllers.Controller;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.CommandBinder;
import frc.robot.subsystems.PickupSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {

  private static final class Constants {
    private static final class Controller {
      private static final double deadband = 0.1;
      private static final int operatorPort = 1;
    }
  }

  public RobotContainer() {
    Controller operator = Controller.createFromXBox
        .apply(Constants.Controller.operatorPort)
        .apply(Constants.Controller.deadband);

    SmartDashboard.putData("Operator", operator);

    ShooterSubsystem shooterSubsystem = ShooterSubsystem.create.get();
    PickupSubsystem pickupSubsystem = PickupSubsystem.create.get();

    CommandBinder.bindManualShooterCommand
        .apply(pickupSubsystem)
        .apply(shooterSubsystem)
        .accept(operator.b);

    CommandBinder.bindManualPickupCommand
        .apply(pickupSubsystem)
        .apply(shooterSubsystem)
        .accept(operator.a);

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
