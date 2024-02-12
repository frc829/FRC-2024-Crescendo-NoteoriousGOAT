package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Value;
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
import frc.robot.RobotContainer;

public class ShooterSubsystem extends SubsystemBase {

  private static final class Constants {
    private static final class Top {
      private static final int deviceId = 16;
      private static final double slot0kP = 0.0;
      private static final double slot0kI = 0.0;
      private static final double slot0kD = 0.0;
      private static final double slot0kF = 1.0
          / Units.radiansPerSecondToRotationsPerMinute(DCMotor.getNeoVortex(1).freeSpeedRadPerSec);
    }

    private static final class Bottom {
      private static final int deviceId = 26;
      private static final double slot0kP = 0.0;
      private static final double slot0kI = 0.0;
      private static final double slot0kD = 0.0;
      private static final double slot0kF = 1.0
          / Units.radiansPerSecondToRotationsPerMinute(DCMotor.getNeoVortex(1).freeSpeedRadPerSec);
    }
  }

  public final Measure<Voltage> topVoltage;
  public final Measure<Voltage> bottomVoltage;
  public final Measure<Dimensionless> topVelocity;
  public final Measure<Dimensionless> bottomVelocity;
  public final Consumer<Double> spinTop;
  public final Consumer<Double> spinBottom;
  public final Runnable update;


  private ShooterSubsystem(
      Measure<Voltage> topVoltage,
      Measure<Voltage> bottomVoltage,
      Measure<Dimensionless> topVelocity,
      Measure<Dimensionless> bottomVelocity,
      Consumer<Double> spinTop,
      Consumer<Double> spinBottom,
      Runnable stop,
      Runnable update) {
    this.topVoltage = topVoltage;
    this.bottomVoltage = bottomVoltage;
    this.topVelocity = topVelocity;
    this.bottomVelocity = bottomVelocity;
    this.spinTop = spinTop;
    this.spinBottom = spinBottom;
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
        "Top Velocity",
        () -> GoatMath.round(topVelocity.in(Percent), 2),
        null);

    builder.addDoubleProperty(
        "Top Voltage",
        () -> GoatMath.round(topVoltage.in(Volts), 2),
        null);

    builder.addDoubleProperty(
        "Bottom Velocity",
        () -> GoatMath.round(bottomVelocity.in(Percent), 2),
        null);

    builder.addDoubleProperty(
        "Bottom Voltage",
        () -> GoatMath.round(bottomVoltage.in(Volts), 2),
        null);
  }

  public static final Supplier<ShooterSubsystem> create = () -> {
    Spinner top = Motor.REV.createCANSparkBaseNEOVortex
        .andThen(Motor.REV.setkP.apply(0).apply(Constants.Top.slot0kP))
        .andThen(Motor.REV.setkI.apply(0).apply(Constants.Top.slot0kI))
        .andThen(Motor.REV.setkD.apply(0).apply(Constants.Top.slot0kD))
        .andThen(Motor.REV.setkF.apply(0).apply(Constants.Top.slot0kF))
        .andThen(Motor.REV.enableCoast)
        .andThen(Motor.REV.createMotorFromCANSparkBase.apply(1.0))
        .andThen(Motor.REV.setNEOVortexMaxVelocity)
        .andThen(Motor.REV.setSpinSim)
        .andThen(Spinner.create)
        .apply(Constants.Top.deviceId);

    Spinner bottom = Motor.REV.createCANSparkBaseNEOVortex
        .andThen(Motor.REV.setkP.apply(0).apply(Constants.Bottom.slot0kP))
        .andThen(Motor.REV.setkI.apply(0).apply(Constants.Bottom.slot0kI))
        .andThen(Motor.REV.setkD.apply(0).apply(Constants.Bottom.slot0kD))
        .andThen(Motor.REV.setkF.apply(0).apply(Constants.Bottom.slot0kF))
        .andThen(Motor.REV.enableCoast)
        .andThen(Motor.REV.createMotorFromCANSparkBase.apply(1.0))
        .andThen(Motor.REV.setNEOVortexMaxVelocity)
        .andThen(Motor.REV.setSpinSim)
        .andThen(Spinner.create)
        .apply(Constants.Bottom.deviceId);

    Runnable stop = () -> {
      top.stop.run();
      bottom.stop.run();
    };

    Runnable update = () -> {
      top.update.run();
      bottom.update.run();
    };

    return new ShooterSubsystem(
        top.voltage,
        bottom.voltage,
        top.velocity,
        bottom.velocity,
        top.spin,
        bottom.spin,
        stop,
        update);

  };
}
