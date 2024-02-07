// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.controllers.Controller;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.CommandBinder;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PickupSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterTiltSubsystem;

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

    PickupSubsystem pickupSubsystem = PickupSubsystem.create.get();
    ShooterSubsystem shooterSubsystem = ShooterSubsystem.create.get();
    ShooterTiltSubsystem shooterTiltSubsystem = ShooterTiltSubsystem.create.get();
    ElevatorSubsystem elevatorSubsystem = ElevatorSubsystem.create.get();

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

    SmartDashboard.putData("Operator", operator);
    SmartDashboard.putData("Pickup", pickupSubsystem);
    SmartDashboard.putData("Shooter", shooterSubsystem);
    SmartDashboard.putData("Shooter Tilt", shooterTiltSubsystem);
    SmartDashboard.putData("Elevator", elevatorSubsystem);

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
