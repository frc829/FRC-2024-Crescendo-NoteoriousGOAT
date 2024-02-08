package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Value;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.Supplier;

import com.compLevel0.Motor;
import com.compLevel1.Turner;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Dimensionless;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterTiltSubsystem extends SubsystemBase {

  private static final class Constants {
    private static final int deviceId = 18;
    private static final double gearing = 5.0 * 4.0 * 3.0 * 56.0 / 18.0;
    private static final double slot0kP = 0.0;
    private static final double slot0kI = 0.0;
    private static final double slot0kD = 0.0;
    private static final double slot0kF = 1.0
        / Units.radiansPerSecondToRotationsPerMinute(DCMotor.getNEO(1).freeSpeedRadPerSec);
    private static final double slot1kP = 2;
    private static final double slot1kI = 0.0;
    private static final double slot1kD = 0.0;
    private static final double slot1kF = 0.0;
    private static final Measure<Angle> absoluteEncoderOffset = Degrees.of(0); // TODO:
    private static final double absoluteAngleScaleFactor = 1.0; // TODO:
  }

  public final Measure<Voltage> voltage;
  public final Measure<Angle> angle;
  public final Measure<Angle> absoluteAngle;
  public final Measure<Dimensionless> velocity;
  public final Runnable update;

  public final Supplier<Command> createHoldCommand;
  public final Function<DoubleSupplier, Command> createDriveTiltCommand;
  public final Function<Supplier<Measure<Angle>>, Command> createTurnTiltCommand;

  private ShooterTiltSubsystem(
      Measure<Voltage> voltage,
      Measure<Angle> angle,
      Measure<Angle> absoluteAngle,
      Measure<Dimensionless> velocity,
      Consumer<Measure<Angle>> turn,
      Consumer<Double> drive,
      Runnable resetRelEncoderFromAbsolute,
      Runnable hold,
      Runnable update) {
    this.voltage = voltage;
    this.angle = angle;
    this.absoluteAngle = absoluteAngle;
    this.velocity = velocity;
    this.update = update;

    resetRelEncoderFromAbsolute.run();

    createHoldCommand = () -> {
      Command holdCommand = run(hold);
      holdCommand.setName("HOLD");
      return holdCommand;
    };

    createDriveTiltCommand = (setpoint) -> {
      Runnable driveTilt = () -> drive.accept(setpoint.getAsDouble());
      Command driveTiltCommand = run(driveTilt);
      driveTiltCommand.setName("Drive Tilt Control");
      return driveTiltCommand;
    };

    createTurnTiltCommand = (setpoint) -> {
      Runnable turnTilt = () -> turn.accept(setpoint.get());
      Command turnTiltCommand = run(turnTilt);
      turnTiltCommand.setName("Turn Tilt Control");
      return turnTiltCommand;
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
        "Angle",
        () -> MathUtil.inputModulus(angle.in(Degrees), -180, 180),
        null);

    builder.addDoubleProperty(
        "Absolute Angle",
        () -> absoluteAngle.in(Degrees),
        null);

    builder.addDoubleProperty(
        "Velocity",
        () -> velocity.in(Value),
        null);
  }

  public static final Supplier<ShooterTiltSubsystem> create = () -> {
    Turner turner = Motor.REV.createCANSparkBaseNEO
        .andThen(Motor.REV.setkP.apply(0).apply(Constants.slot0kP))
        .andThen(Motor.REV.setkI.apply(0).apply(Constants.slot0kI))
        .andThen(Motor.REV.setkD.apply(0).apply(Constants.slot0kD))
        .andThen(Motor.REV.setkF.apply(0).apply(Constants.slot0kF))
        .andThen(Motor.REV.setkP.apply(1).apply(Constants.slot1kP))
        .andThen(Motor.REV.setkI.apply(1).apply(Constants.slot1kI))
        .andThen(Motor.REV.setkD.apply(1).apply(Constants.slot1kD))
        .andThen(Motor.REV.setkF.apply(1).apply(Constants.slot1kF))
        .andThen(Motor.REV.setAngleWrapping.apply(Constants.gearing))
        .andThen(Motor.REV.setAbsolutEencoderScaleFactor.apply(Constants.absoluteAngleScaleFactor))
        .andThen(Motor.REV.setInvertAbsoluteEncoder)
        .andThen(Motor.REV.setAbsoluteEncoderOffset.apply(Constants.absoluteEncoderOffset))
        .andThen(Motor.REV.createMotorFromCANSparkBase.apply(Constants.gearing))
        .andThen(Motor.REV.setNEOMaxVelocity)
        .andThen(Motor.REV.setTurnSim
            .apply(Constants.slot1kP)
            .apply(Constants.slot1kI)
            .apply(Constants.slot1kD)
            .apply(true)
            .apply(Constants.gearing))
        .andThen(Motor.REV.setSpinSim)
        .andThen(Turner.create.apply(Constants.gearing))
        .apply(Constants.deviceId);

    return new ShooterTiltSubsystem(
        turner.voltage,
        turner.angle,
        turner.absoluteAngle,
        turner.velocity,
        turner.turn,
        turner.drive,
        turner.resetRelEncoderFromAbsolute,
        turner.hold,
        turner.update);
  };
}
