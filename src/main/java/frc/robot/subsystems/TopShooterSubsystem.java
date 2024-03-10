package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Consumer;
import java.util.function.Supplier;

import com.compLevel0.Motor;
import com.compLevel1.Spinner;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.utility.GoatMath;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Dimensionless;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TopShooterSubsystem extends SubsystemBase {

  private static final class Constants {
    private static final int deviceId = 16;
    private static final double slot0kP = 0.0;
    private static final double slot0kI = 0.0;
    private static final double slot0kD = 0.0;
    private static final double slot0kF = 1.0
        / Units.radiansPerSecondToRotationsPerMinute(DCMotor.getNeoVortex(1).freeSpeedRadPerSec);
  }

  public final Measure<Voltage> voltage;
  public final Measure<Dimensionless> velocity;
  public final Consumer<Double> spin;
  public final Runnable stop;
  public final Runnable update;

  private TopShooterSubsystem(
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

  public static final Supplier<TopShooterSubsystem> create = () -> {
    CANSparkBase canSparkBase;
    if (RobotBase.isSimulation()) {
      CANSparkMax canSparkMax = new CANSparkMax(Constants.deviceId, MotorType.kBrushless);
      canSparkBase = canSparkMax;
      REVPhysicsSim.getInstance().addSparkMax(canSparkMax, DCMotor.getNeoVortex(1));
    } else {
      canSparkBase = new CANSparkFlex(Constants.deviceId, MotorType.kBrushless);
    }

    canSparkBase.setIdleMode(IdleMode.kCoast);
    canSparkBase.getPIDController().setP(Constants.slot0kP, 0);
    canSparkBase.getPIDController().setI(Constants.slot0kI, 0);
    canSparkBase.getPIDController().setD(Constants.slot0kD, 0);
    canSparkBase.getPIDController().setFF(Constants.slot0kF, 0);
    canSparkBase.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);

    Spinner spinner = Motor.REV.createNEOVortexMotor
        .andThen(Spinner.create)
        .apply(canSparkBase);

    return new TopShooterSubsystem(
        spinner.voltage,
        spinner.velocity,
        spinner.spin,
        spinner.stop,
        spinner.update);

  };
}
