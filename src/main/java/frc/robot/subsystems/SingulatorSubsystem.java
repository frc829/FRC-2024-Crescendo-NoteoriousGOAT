package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Consumer;
import java.util.function.Supplier;

import com.compLevel0.Motor;
import com.compLevel1.Spinner;
import com.utility.GoatMath;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Dimensionless;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SingulatorSubsystem extends SubsystemBase {

  private static final class Constants {
    private static final int deviceId = 25;
    private static final double slot0kP = 0.0;
    private static final double slot0kI = 0.0;
    private static final double slot0kD = 0.0;
    private static final double slot0kF = 1.0
        / Units.radiansPerSecondToRotationsPerMinute(DCMotor.getNeo550(1).freeSpeedRadPerSec);
  }

  public final Measure<Voltage> voltage;
  public final Measure<Dimensionless> velocity;
  public final Consumer<Double> spin;
  public final Runnable stop;
  public final Runnable update;

  private SingulatorSubsystem(
      Measure<Voltage> voltage,
      Measure<Dimensionless> velocity,
      Consumer<Double> spin,
      Runnable stop,
      Runnable update) {
    this.voltage = voltage;
    this.velocity = velocity;
    this.spin = spin;
    this.stop = stop;
    this.update = update;

    Command defaultCommand = run(stop);
    defaultCommand.setName("STOP");
    this.setDefaultCommand(defaultCommand);

  }

  @Override
  public void periodic() {
    update.run();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty(
        "Velocity",
        () -> GoatMath.round(velocity.in(Percent), 2),
        null);

    builder.addDoubleProperty(
        "Voltage",
        () -> GoatMath.round(voltage.in(Volts), 2),
        null);
  }

  public static final Supplier<SingulatorSubsystem> create = () -> {
    Spinner spinner = Motor.REV.createCANSparkBaseNEO550
        .andThen(Motor.REV.setkP.apply(0).apply(Constants.slot0kP))
        .andThen(Motor.REV.setkI.apply(0).apply(Constants.slot0kI))
        .andThen(Motor.REV.setkD.apply(0).apply(Constants.slot0kD))
        .andThen(Motor.REV.setkF.apply(0).apply(Constants.slot0kF))
        .andThen(Motor.REV.enableBrake)
        .andThen(Motor.REV.createMotorFromCANSparkBase)
        .andThen(Motor.REV.setNEO550MaxVelocity)
        .andThen(Spinner.create)
        .apply(Constants.deviceId);

    return new SingulatorSubsystem(
        spinner.voltage,
        spinner.velocity,
        spinner.spin,
        spinner.stop,
        spinner.update);
  };
}
