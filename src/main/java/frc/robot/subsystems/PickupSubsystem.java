package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.Supplier;

import com.compLevel0.Motor;
import com.compLevel0.Sensor;
import com.compLevel1.PositionSwitch;
import com.compLevel1.Spinner;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Dimensionless;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PickupSubsystem extends SubsystemBase {

  private static final class Constants {
    private static final class Outer {
      private static final int deviceId = 14;
      private static final double slot0kP = 0.0;
      private static final double slot0kI = 0.0;
      private static final double slot0kD = 0.0;
      private static final double slot0kF = 1.0
          / Units.radiansPerSecondToRotationsPerMinute(DCMotor.getNeo550(1).freeSpeedRadPerSec);
    }

    private static final class Inner {
      private static final int deviceId = 15;
      private static final double slot0kP = 0.0;
      private static final double slot0kI = 0.0;
      private static final double slot0kD = 0.0;
      private static final double slot0kF = 1.0
          / Units.radiansPerSecondToRotationsPerMinute(DCMotor.getNeo550(1).freeSpeedRadPerSec);
    }

    private static final class Transport {
      private static final int deviceId = 24;
      private static final double slot0kP = 0.0;
      private static final double slot0kI = 0.0;
      private static final double slot0kD = 0.0;
      private static final double slot0kF = 1.0
          / Units.radiansPerSecondToRotationsPerMinute(DCMotor.getNeo550(1).freeSpeedRadPerSec);
    }

    private static final class Singulator {
      private static final int deviceId = 25;
      private static final double slot0kP = 0.0;
      private static final double slot0kI = 0.0;
      private static final double slot0kD = 0.0;
      private static final double slot0kF = 1.0
          / Units.radiansPerSecondToRotationsPerMinute(DCMotor.getNeo550(1).freeSpeedRadPerSec);
    }

    private static final class PositionSwitch {
      private static final int deviceId = 34;
      private static final Measure<Distance> minOn = Inches.of(0);
      private static final Measure<Distance> maxOn = Inches.of(0.5);
    }
  }

  public final Measure<Voltage> outerVoltage;
  public final Measure<Voltage> innerVoltage;
  public final Measure<Voltage> transportVoltage;
  public final Measure<Voltage> singulatorVoltage;
  public final Measure<Dimensionless> outerVelocity;
  public final Measure<Dimensionless> innerVelocity;
  public final Measure<Dimensionless> transportVelocity;
  public final Measure<Dimensionless> singulatorVelocity;
  public final BooleanSupplier hasNote;
  public final Consumer<Double> spinOuter;
  public final Consumer<Double> spinInner;
  public final Consumer<Double> spinTransport;
  public final Consumer<Double> spinSingulator;
  public final Runnable update;

  public final Supplier<Command> createStopCommand;
  public final Function<DoubleSupplier, Function<DoubleSupplier, Function<DoubleSupplier, Function<DoubleSupplier, Command>>>> createPickupControlCommand;

  private PickupSubsystem(
      Measure<Voltage> outerVoltage,
      Measure<Voltage> innerVoltage,
      Measure<Voltage> transportVoltage,
      Measure<Voltage> singulatorVoltage,
      Measure<Dimensionless> outerVelocity,
      Measure<Dimensionless> innerVelocity,
      Measure<Dimensionless> transportVelocity,
      Measure<Dimensionless> singulatorVelocity,
      BooleanSupplier hasNote,
      Consumer<Double> spinOuter,
      Consumer<Double> spinInner,
      Consumer<Double> spinTransport,
      Consumer<Double> spinSingulator,
      Runnable stop,
      Runnable update) {
    this.outerVoltage = outerVoltage;
    this.innerVoltage = innerVoltage;
    this.transportVoltage = transportVoltage;
    this.singulatorVoltage = singulatorVoltage;
    this.outerVelocity = outerVelocity;
    this.innerVelocity = innerVelocity;
    this.transportVelocity = transportVelocity;
    this.singulatorVelocity = singulatorVelocity;
    this.hasNote = hasNote;
    this.spinOuter = spinOuter;
    this.spinInner = spinInner;
    this.spinTransport = spinTransport;
    this.spinSingulator = spinSingulator;
    this.update = update;

    createStopCommand = () -> {
      Command stopCommand = run(stop);
      stopCommand.setName("STOP");
      return stopCommand;
    };

    createPickupControlCommand = (
        outerSetpoint) -> (innerSetpoint) -> (transportSetpoint) -> (singulatorControl) -> {
          Runnable spin = () -> {
            spinOuter.accept(outerSetpoint.getAsDouble());
            spinInner.accept(innerSetpoint.getAsDouble());
            spinTransport.accept(transportSetpoint.getAsDouble());
            spinSingulator.accept(singulatorControl.getAsDouble());
          };
          Command spinCommand = run(spin);
          spinCommand.setName("Pickup Control");
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

  public static final Supplier<PickupSubsystem> create = () -> {
    Spinner outer = Motor.REV.createCANSparkBaseNEO550
        .andThen(Motor.REV.setkP.apply(0).apply(Constants.Outer.slot0kP))
        .andThen(Motor.REV.setkI.apply(0).apply(Constants.Outer.slot0kI))
        .andThen(Motor.REV.setkD.apply(0).apply(Constants.Outer.slot0kD))
        .andThen(Motor.REV.setkF.apply(0).apply(Constants.Outer.slot0kF))
        .andThen(Motor.REV.createMotorFromCANSparkBase)
        .andThen(Motor.REV.setNEO550MaxVelocity)
        .andThen(Motor.REV.setSpinSim)
        .andThen(Spinner.create)
        .apply(Constants.Outer.deviceId);

    Spinner inner = Motor.REV.createCANSparkBaseNEO550
        .andThen(Motor.REV.setkP.apply(0).apply(Constants.Inner.slot0kP))
        .andThen(Motor.REV.setkI.apply(0).apply(Constants.Inner.slot0kI))
        .andThen(Motor.REV.setkD.apply(0).apply(Constants.Inner.slot0kD))
        .andThen(Motor.REV.setkF.apply(0).apply(Constants.Inner.slot0kF))
        .andThen(Motor.REV.createMotorFromCANSparkBase)
        .andThen(Motor.REV.setNEO550MaxVelocity)
        .andThen(Motor.REV.setSpinSim)
        .andThen(Spinner.create)
        .apply(Constants.Inner.deviceId);

    Spinner transport = Motor.REV.createCANSparkBaseNEO550
        .andThen(Motor.REV.setkP.apply(0).apply(Constants.Transport.slot0kP))
        .andThen(Motor.REV.setkI.apply(0).apply(Constants.Transport.slot0kI))
        .andThen(Motor.REV.setkD.apply(0).apply(Constants.Transport.slot0kD))
        .andThen(Motor.REV.setkF.apply(0).apply(Constants.Transport.slot0kF))
        .andThen(Motor.REV.createMotorFromCANSparkBase)
        .andThen(Motor.REV.setNEO550MaxVelocity)
        .andThen(Motor.REV.setSpinSim)
        .andThen(Spinner.create)
        .apply(Constants.Transport.deviceId);

    Spinner singulator = Motor.REV.createCANSparkBaseNEO550
        .andThen(Motor.REV.setkP.apply(0).apply(Constants.Singulator.slot0kP))
        .andThen(Motor.REV.setkI.apply(0).apply(Constants.Singulator.slot0kI))
        .andThen(Motor.REV.setkD.apply(0).apply(Constants.Singulator.slot0kD))
        .andThen(Motor.REV.setkF.apply(0).apply(Constants.Singulator.slot0kF))
        .andThen(Motor.REV.createMotorFromCANSparkBase)
        .andThen(Motor.REV.setNEO550MaxVelocity)
        .andThen(Motor.REV.setSpinSim)
        .andThen(Spinner.create)
        .apply(Constants.Singulator.deviceId);

    PositionSwitch positionSwitch = Sensor.PWF.createTimeOfFlight
        .andThen(Sensor.PWF.createDistanceSensor)
        .andThen(PositionSwitch.create)
        .apply(Constants.PositionSwitch.deviceId)
        .apply(Constants.PositionSwitch.minOn)
        .apply(Constants.PositionSwitch.maxOn);

    Runnable stop = () -> {
      outer.stop.run();
      inner.stop.run();
      transport.stop.run();
      singulator.stop.run();
    };

    Runnable update = () -> {
      outer.update.run();
      inner.update.run();
      transport.update.run();
      singulator.update.run();
    };

    return new PickupSubsystem(
        outer.voltage,
        inner.voltage,
        transport.voltage,
        singulator.voltage,
        outer.velocity,
        inner.velocity,
        transport.velocity,
        singulator.velocity,
        positionSwitch.isOn,
        outer.spin,
        inner.spin,
        transport.spin,
        singulator.spin,
        stop,
        update);

  };
}
