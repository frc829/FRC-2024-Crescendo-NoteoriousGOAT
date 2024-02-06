package frc.robot.subsystems;

import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.Supplier;

import com.compLevel0.Motor;
import com.compLevel1.Spinner;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Dimensionless;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
  public final Runnable update;

  public final Supplier<Command> createStopCommand;
  public final Function<DoubleSupplier, Function<DoubleSupplier, Command>> createSpinShootersCommand;

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
    this.update = update;

    createStopCommand = () -> {
      Command stopCommand = run(stop);
      stopCommand.setName("STOP");
      return stopCommand;
    };

    createSpinShootersCommand = (topSetpoint) -> (bottomSetpoint) -> {
      Runnable spin = () -> {
        spinTop.accept(topSetpoint.getAsDouble());
        spinBottom.accept(bottomSetpoint.getAsDouble());
      };
      Command spinCommand = run(spin);
      spinCommand.setName("Shooter Control");
      return spinCommand;
    };

    this.setDefaultCommand(createStopCommand.get());

  }

  @Override
  public void periodic() {
    update.run();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
  }

  public static final Supplier<ShooterSubsystem> create = () -> {
    Spinner top = Motor.REV.createCANSparkBaseNEOVortex
        .andThen(Motor.REV.setkP.apply(0).apply(Constants.Top.slot0kP))
        .andThen(Motor.REV.setkI.apply(0).apply(Constants.Top.slot0kI))
        .andThen(Motor.REV.setkD.apply(0).apply(Constants.Top.slot0kD))
        .andThen(Motor.REV.setkF.apply(0).apply(Constants.Top.slot0kF))
        .andThen(Motor.REV.createMotorFromCANSparkBase)
        .andThen(Motor.REV.setNEOVortexMaxVelocity)
        .andThen(Spinner.create)
        .apply(Constants.Top.deviceId);

    Spinner bottom = Motor.REV.createCANSparkBaseNEOVortex
        .andThen(Motor.REV.setkP.apply(0).apply(Constants.Bottom.slot0kP))
        .andThen(Motor.REV.setkI.apply(0).apply(Constants.Bottom.slot0kI))
        .andThen(Motor.REV.setkD.apply(0).apply(Constants.Bottom.slot0kD))
        .andThen(Motor.REV.setkF.apply(0).apply(Constants.Bottom.slot0kF))
        .andThen(Motor.REV.createMotorFromCANSparkBase)
        .andThen(Motor.REV.setNEOVortexMaxVelocity)
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
