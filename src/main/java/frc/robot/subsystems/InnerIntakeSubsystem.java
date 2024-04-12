package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Value;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.VoltsPerRadianPerSecond;
import static edu.wpi.first.units.Units.VoltsPerRadianPerSecondSquared;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import com.simulation.REVMotorSimulation;
import com.utility.GoatMath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Current;
import edu.wpi.first.units.Dimensionless;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Per;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class InnerIntakeSubsystem extends SubsystemBase {

  private static final class Constants {
    private static final int deviceId = 15;
    private static final MotorType type = MotorType.kBrushless;
    private static final Measure<Voltage> kS = Volts.of(0.082167);
    private static final double kVVoltsPerRPM = 0.0010505;
    private static final Measure<Per<Voltage, Velocity<Angle>>> kV = VoltsPerRadianPerSecond
        .of(kVVoltsPerRPM * 60 / 2 / Math.PI);
    private static final double kAVoltsPerRPMSq = 7.8344e-05;
    private static final Measure<Per<Voltage, Velocity<Velocity<Angle>>>> kA = VoltsPerRadianPerSecondSquared
        .of(kAVoltsPerRPMSq * 3600 / 2 / Math.PI);
    private static final LinearSystem<N2, N1, N2> plant = LinearSystemId
        .createDCMotorSystem(kV.in(VoltsPerRadianPerSecond), kAVoltsPerRPMSq);
    private static final double gearing = 1.0;
    private static final DCMotor gearbox = DCMotor.getNeo550(1);
    private static final SimpleMotorFeedforward simpleMotorFeedforward = new SimpleMotorFeedforward(
        kS.in(Volts),
        kV.in(VoltsPerRadianPerSecond),
        kA.in(VoltsPerRadianPerSecondSquared));
    private static final Measure<Velocity<Angle>> maxVelocity = RadiansPerSecond
        .of(simpleMotorFeedforward.maxAchievableVelocity(12.0, 0));
  }

  private final MutableMeasure<Voltage> voltage;
  private final MutableMeasure<Dimensionless> velocityRatio;
  private final MutableMeasure<Current> current;

  private final CANSparkMax canSparkMax;
  private final REVMotorSimulation revSimulation;

  public InnerIntakeSubsystem() {
    voltage = MutableMeasure.zero(Volts);
    velocityRatio = MutableMeasure.zero(Value);
    current = MutableMeasure.zero(Amps);

    canSparkMax = new CANSparkMax(Constants.deviceId, Constants.type);
    canSparkMax.getPIDController().setFF(Constants.kVVoltsPerRPM);
    canSparkMax.setIdleMode(IdleMode.kBrake);
    canSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
    revSimulation = new REVMotorSimulation(
        Constants.deviceId,
        Constants.gearbox.stallTorqueNewtonMeters,
        Constants.gearbox.freeSpeedRadPerSec * 60 / 2 / Math.PI,
        Constants.plant,
        Constants.gearbox,
        Constants.gearing,
        Constants.kS);

  }

  public void updateTelemetry() {
    voltage.mut_setMagnitude(canSparkMax.getAppliedOutput() * canSparkMax.getBusVoltage());
    velocityRatio.mut_setMagnitude(canSparkMax.getEncoder().getVelocity() / Constants.maxVelocity.in(RPM));
    current.mut_setMagnitude(canSparkMax.getOutputCurrent());
  }

  @Override
  public void periodic() {
    updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    revSimulation.update();
  }

  private void setVelocityRatio(double velocityRatio) {
    if (RobotBase.isReal()) {
      double arbFeedforward = velocityRatio != 0
          ? Constants.kS.in(Volts) * Math.signum(velocityRatio)
          : 0.0;
      canSparkMax.getPIDController().setReference(
          velocityRatio * Constants.maxVelocity.in(RPM),
          ControlType.kVelocity,
          0,
          arbFeedforward,
          ArbFFUnits.kVoltage);
    } else {
      double voltage = Constants.simpleMotorFeedforward
          .calculate(velocityRatio * Constants.maxVelocity.in(RadiansPerSecond), 0);
      voltage = MathUtil.clamp(voltage, -RobotController.getBatteryVoltage(), RobotController.getBatteryVoltage());
      this.voltage.mut_setMagnitude(voltage);
      revSimulation.setInputVoltage(voltage);
    }
  }

  private void stop() {
    if (RobotBase.isReal()) {
      canSparkMax.stopMotor();
    } else {
      revSimulation.setInputVoltage(0.0);
    }
  }

  public Command createSetVelocityRatioCommand(DoubleSupplier velocityRatioSupplier) {
    return run(() -> setVelocityRatio(velocityRatioSupplier.getAsDouble()))
        .withName("Set Velocity Ratio");
  }

  public Command createStopCommand() {
    return run(this::stop).withName("STOP");
  }

  public boolean atSpeed(double velocityRatio, double velocityRatioTolerance) {
    return MathUtil.isNear(velocityRatio, this.velocityRatio.in(Value), velocityRatioTolerance);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty(
        "Velocity % of Max",
        () -> GoatMath.round(velocityRatio.in(Percent), 2),
        null);

    builder.addDoubleProperty(
        "Voltage (V)",
        () -> GoatMath.round(voltage.in(Volts), 2),
        null);

    builder.addDoubleProperty(
        "Current (A)",
        () -> GoatMath.round(current.in(Amps), 2),
        null);
  }
}
