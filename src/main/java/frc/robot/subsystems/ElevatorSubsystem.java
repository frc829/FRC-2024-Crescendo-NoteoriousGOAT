package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Value;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.Supplier;

import com.compLevel0.Motor;
import com.compLevel1.Elevator;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Dimensionless;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {

  private static final class Constants {
    private static final int deviceId = 28;
    private static final double gearing = 4.0 * 4.0 * 4.0 * 30.0 / 40.0;
    private static final Measure<Distance> drumRadius = Inches.of(2.0);
    private static final double slot0kP = 0.0;
    private static final double slot0kI = 0.0;
    private static final double slot0kD = 0.0;
    private static final double slot0kF = 1.0
        / Units.radiansPerSecondToRotationsPerMinute(DCMotor.getNEO(1).freeSpeedRadPerSec);
    private static final double slot1kP = 0.5;
    private static final double slot1kI = 0.0;
    private static final double slot1kD = 0.0;
    private static final double slot1kF = 0.0;
  }

  public final Measure<Voltage> voltage;
  public final Measure<Distance> position;
  public final Measure<Dimensionless> velocity;
  public final Runnable update;

  public final Supplier<Command> createHoldCommand;
  public final Function<DoubleSupplier, Command> createDriveElevatorcommand;
  public final Function<Supplier<Measure<Distance>>, Command> createMoveElevatorCommand;

  private ElevatorSubsystem(
      Measure<Voltage> voltage,
      Measure<Distance> position,
      Measure<Dimensionless> velocity,
      Consumer<Measure<Distance>> move,
      Consumer<Double> drive,
      Runnable hold,
      Runnable update) {
    this.voltage = voltage;
    this.position = position;
    this.velocity = velocity;
    this.update = update;

    createHoldCommand = () -> {
      Command holdCommand = run(hold);
      holdCommand.setName("HOLD");
      return holdCommand;
    };

    createDriveElevatorcommand = (setpoint) -> {
      Runnable driveElevator = () -> drive.accept(setpoint.getAsDouble());
      Command driveElevatorCommand = run(driveElevator);
      driveElevatorCommand.setName("Drive Elevator Control");
      return driveElevatorCommand;
    };

    createMoveElevatorCommand = (setpoint) -> {
      Runnable moveElevator = () -> move.accept(setpoint.get());
      Command moveElevatorCommand = run(moveElevator);
      moveElevatorCommand.setName("Move Elevator Control");
      return moveElevatorCommand;
    };

    this.setDefaultCommand(createHoldCommand.get());
  }

  @Override
  public void periodic() {
    update.run();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty(
        "Voltage",
        () -> voltage.in(Volts),
        null);

    builder.addDoubleProperty(
        "Position",
        () -> position.in(Meters),
        null);

    builder.addDoubleProperty(
        "Velocity",
        () -> velocity.in(Value),
        null);
  }

  public static final Supplier<ElevatorSubsystem> create = () -> {
    Elevator elevator = Motor.REV.createCANSparkBaseNEO
        .andThen(Motor.REV.setkP.apply(0).apply(Constants.slot0kP))
        .andThen(Motor.REV.setkI.apply(0).apply(Constants.slot0kI))
        .andThen(Motor.REV.setkD.apply(0).apply(Constants.slot0kD))
        .andThen(Motor.REV.setkF.apply(0).apply(Constants.slot0kF))
        .andThen(Motor.REV.setkP.apply(1).apply(Constants.slot1kP))
        .andThen(Motor.REV.setkI.apply(1).apply(Constants.slot1kI))
        .andThen(Motor.REV.setkD.apply(1).apply(Constants.slot1kD))
        .andThen(Motor.REV.setkF.apply(1).apply(Constants.slot1kF))
        .andThen(Motor.REV.createMotorFromCANSparkBase.apply(Constants.gearing))
        .andThen(Motor.REV.setNEOMaxVelocity)
        .andThen(Motor.REV.setTurnSim
            .apply(Constants.slot1kP)
            .apply(Constants.slot1kI)
            .apply(Constants.slot1kD)
            .apply(false)
            .apply(Double.NaN))
        .andThen(Motor.REV.setSpinSim)
        .andThen(Elevator.create
            .apply(Constants.gearing)
            .apply(Constants.drumRadius))
        .apply(Constants.deviceId);

    return new ElevatorSubsystem(
        elevator.voltage,
        elevator.position,
        elevator.velocity,
        elevator.move,
        elevator.drive,
        elevator.hold,
        elevator.update);

  };
}
