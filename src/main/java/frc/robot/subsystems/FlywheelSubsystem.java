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
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.simulation.NewDCMotorSim;
import com.simulation.REVMotorSimulation;
import com.utility.GoatMath;

import edu.wpi.first.math.MathUtil;
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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class FlywheelSubsystem extends SubsystemBase {

  @SuppressWarnings("resource")
  public static final FlywheelSubsystem createNEOVortex(
      int deviceId,
      Measure<Voltage> kS,
      Measure<Per<Voltage, Velocity<Angle>>> kV,
      Measure<Per<Voltage, Velocity<Velocity<Angle>>>> kA,
      double gearing,
      double dtSeconds) {

    MutableMeasure<Voltage> voltage = MutableMeasure.zero(Volts);
    MutableMeasure<Dimensionless> velocityRatio = MutableMeasure.zero(Value);
    MutableMeasure<Current> current = MutableMeasure.zero(Amps);

    CANSparkFlex canSparkFlex = new CANSparkFlex(deviceId, MotorType.kBrushless);
    REVMotorSimulation revMotorSimulation = new REVMotorSimulation(
        deviceId,
        kS,
        kV,
        kA,
        DCMotor.getNeoVortex(1),
        gearing,
        dtSeconds);
    Measure<Velocity<Angle>> maxVelocity = RadiansPerSecond.of(DCMotor.getNeoVortex(1).freeSpeedRadPerSec);
    return new FlywheelSubsystem(
        maxVelocity,
        voltage,
        current,
        velocityRatio) {

      @Override
      public void updateData() {
        voltage.mut_setMagnitude(canSparkFlex.getAppliedOutput() * canSparkFlex.getBusVoltage());
        velocityRatio.mut_setMagnitude(canSparkFlex.getEncoder().getVelocity() / maxVelocity.in(RPM));
        current.mut_setMagnitude(canSparkFlex.getOutputCurrent());
      }

      @Override
      public void spinToSetpoint() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'spinToSetpoint'");
      }

    };
  }

  private final Measure<Velocity<Angle>> maxVelocity;
  private final Measure<Voltage> voltage;
  private final Measure<Dimensionless> velocityRatio;
  private final Measure<Current> current;

  private final MutableMeasure<Dimensionless> velocityRatioSetpoint = MutableMeasure.zero(Value);
  private final MutableMeasure<Velocity<Angle>> velocitySetpoint = MutableMeasure.zero(RadiansPerSecond);

  private FlywheelSubsystem(
      Measure<Velocity<Angle>> maxVelocity,
      Measure<Voltage> voltage,
      Measure<Current> current,
      Measure<Dimensionless> velocityRatio) {
    this.maxVelocity = maxVelocity;
    this.voltage = voltage;
    this.current = current;
    this.velocityRatio = velocityRatio;
  }

  public abstract void updateData();

  @Override
  public void periodic() {
    updateData();
  }

  abstract public void spinToSetpoint();

  private void setVelocity(double velocityRatio) {
    velocityRatioSetpoint.mut_setMagnitude(velocityRatio);
    velocitySetpoint.mut_setMagnitude(velocityRatio * maxVelocity.in(RadiansPerSecond));
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
