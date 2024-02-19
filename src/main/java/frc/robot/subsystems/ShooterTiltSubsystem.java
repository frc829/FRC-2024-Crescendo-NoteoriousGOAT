package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Consumer;
import java.util.function.Supplier;

import com.compLevel0.Motor;
import com.compLevel1.Turner;
import com.utility.GoatMath;

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
    private static final double absoluteGearingUp = 5 * 4 * 3.0;
    private static final double absoluteGearingDown = 56.0 / 18.0;
    private static final double slot0kP = 0.0;
    private static final double slot0kI = 0.0;
    private static final double slot0kD = 0.0;
    private static final double slot0kF = 1.0
        / Units.radiansPerSecondToRotationsPerMinute(DCMotor.getNEO(1).freeSpeedRadPerSec);
    private static final double slot1kP = 2;
    private static final double slot1kI = 0.0;
    private static final double slot1kD = 0.0;
    private static final double slot1kF = 0.0;
  }

  public final Measure<Voltage> voltage;
  public final Measure<Angle> angle;
  public final Measure<Angle> absoluteAngle;
  public final Measure<Dimensionless> velocity;
  public final Consumer<Measure<Angle>> turn;
  public final Consumer<Double> drive;
  public final Runnable resetEncoder;
  public final Runnable hold;
  public final Runnable stop; 
  public final Runnable update;


  private ShooterTiltSubsystem(
      Measure<Voltage> voltage,
      Measure<Angle> angle,
      Measure<Angle> absoluteAngle,
      Measure<Dimensionless> velocity,
      Consumer<Measure<Angle>> turn,
      Consumer<Double> drive,
      Runnable resetRelEncoderFromAbsolute,
      Runnable hold,
      Runnable stop,
      Runnable update) {
    this.voltage = voltage;
    this.angle = angle;
    this.absoluteAngle = absoluteAngle;
    this.velocity = velocity;
    this.turn = turn;
    this.resetEncoder = resetRelEncoderFromAbsolute;
    this.drive = drive;
    this.hold = hold;
    this.update = update;
    this.stop = stop;

    resetRelEncoderFromAbsolute.run();

    Command defaultCommand = run(hold);
    defaultCommand.setName("HOLD");
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
        "Voltage",
        () -> GoatMath.round(voltage.in(Volts), 2),
        null);

    builder.addDoubleProperty(
        "Angle",
        () -> GoatMath.round(MathUtil.inputModulus(angle.in(Degrees), -180, 180), 2),
        null);

    builder.addDoubleProperty(
        "Absolute Angle",
        () -> GoatMath.round(absoluteAngle.in(Degrees), 2),
        null);

    builder.addDoubleProperty(
        "Velocity",
        () -> GoatMath.round(velocity.in(Percent), 2),
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
        .andThen(Motor.REV.enableBrake)
        .andThen(Motor.REV.invert)
        .andThen(Motor.REV.createMotorFromCANSparkBase)
        .andThen(Motor.REV.setNEOMaxVelocity)
        .andThen(Motor.REV.setTurnSim
            .apply(Constants.slot1kP)
            .apply(Constants.slot1kI)
            .apply(Constants.slot1kD)
            .apply(true)
            .apply(Constants.gearing))
        .andThen(Motor.REV.setSpinSim)
        .andThen(Turner.create.apply(Constants.gearing).apply(Constants.absoluteGearingUp).apply(Constants.absoluteGearingDown))
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
        turner.stop,
        turner.update);
  };
}
