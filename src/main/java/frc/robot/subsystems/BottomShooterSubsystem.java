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

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.simulation.REVSimpleSimulation;
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

public class BottomShooterSubsystem extends SubsystemBase {

  private static final class Constants {
    private static final int deviceId = 26;
    private static final MotorType type = MotorType.kBrushless;
    private static final Measure<Voltage> kS = Volts.of(0.082167);
    private static final double kVVoltsPerRPM = 0.0018265;
    private static final Measure<Per<Voltage, Velocity<Angle>>> kV = VoltsPerRadianPerSecond
        .of(kVVoltsPerRPM * 60 / 2 / Math.PI);
    private static final double kAVoltsPerRPMSq = 0.00028717;
    private static final Measure<Per<Voltage, Velocity<Velocity<Angle>>>> kA = VoltsPerRadianPerSecondSquared
        .of(kAVoltsPerRPMSq * 60 / 2 / Math.PI);
    private static final LinearSystem<N2, N1, N2> plant = LinearSystemId
        .createDCMotorSystem(kV.in(VoltsPerRadianPerSecond), kAVoltsPerRPMSq);
    private static final double gearing = 1.0;
    private static final DCMotor gearbox = DCMotor.getNeoVortex(1);
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
  private final MutableMeasure<Dimensionless> velocityRatioSetpoint;
  private final MutableMeasure<Velocity<Angle>> velocitySetpoint;

  private final CANSparkFlex canSparkFlex;
  private final REVSimpleSimulation revSimulation;

  public BottomShooterSubsystem() {
    voltage = MutableMeasure.zero(Volts);
    velocityRatio = MutableMeasure.zero(Value);
    velocityRatioSetpoint = MutableMeasure.zero(Value);
    velocitySetpoint = MutableMeasure.zero(RadiansPerSecond);
    current = MutableMeasure.zero(Amps);

    canSparkFlex = new CANSparkFlex(Constants.deviceId, Constants.type);
    canSparkFlex.setIdleMode(IdleMode.kCoast);
    canSparkFlex.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
    revSimulation = new REVSimpleSimulation(
        0,
        null,
        null,
        null,
        null,
        0,
        0.001);

  }

  public void updateTelemetry() {
    voltage.mut_setMagnitude(canSparkFlex.getAppliedOutput() * canSparkFlex.getBusVoltage());
    velocityRatio.mut_setMagnitude(canSparkFlex.getEncoder().getVelocity() / Constants.maxVelocity.in(RPM));
    current.mut_setMagnitude(canSparkFlex.getOutputCurrent());
  }

  @Override
  public void periodic() {
    updateTelemetry();
  }

  public void update() {
    double voltage = Constants.simpleMotorFeedforward
        .calculate(velocitySetpoint.in(RadiansPerSecond), 0);
    voltage = MathUtil.clamp(voltage, -RobotController.getBatteryVoltage(), RobotController.getBatteryVoltage());
    if (RobotBase.isReal()) {
      canSparkFlex.setVoltage(voltage);
    } else {
      revSimulation.setInputVoltage(voltage);
      revSimulation.update();
    }
  }

  private void setVelocity(double velocityRatio) {
    velocityRatioSetpoint.mut_setMagnitude(velocityRatio);
    velocitySetpoint.mut_setMagnitude(velocityRatio * Constants.maxVelocity.in(RadiansPerSecond));
  }

  public Command createSetVelocityRatioCommand(DoubleSupplier velocityRatioSupplier) {
    return run(() -> setVelocity(velocityRatioSupplier.getAsDouble()))
        .withName("Set Velocity Ratio");
  }

  public Command createStopCommand() {
    return run(() -> setVelocity(0.0)).withName("STOP");
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
