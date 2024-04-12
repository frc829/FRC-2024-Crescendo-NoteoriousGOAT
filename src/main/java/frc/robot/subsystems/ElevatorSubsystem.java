package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Value;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.VoltsPerMeterPerSecond;
import static edu.wpi.first.units.Units.VoltsPerMeterPerSecondSquared;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import com.simulation.REVElevatorSimulation;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.utility.GoatMath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.units.Current;
import edu.wpi.first.units.Dimensionless;
import edu.wpi.first.units.Distance;
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

public class ElevatorSubsystem extends SubsystemBase {

  private static final class Constants {
    private static final int deviceId = 29;
    private static final MotorType type = MotorType.kBrushless;
    private static final double gearing = 4.0 * 4.0 * 3.0 * 40.0 / 30.0;
    private static final Measure<Distance> drumRadius = Inches.of(1.0);
    private static final Measure<Voltage> kS = Volts.of(0.26274);
    private static final Measure<Voltage> kG = Volts.of(0.20027);
    private static final double kVVoltsPerRPM = 0.0020226;
    private static final Measure<Per<Voltage, Velocity<Distance>>> kV = VoltsPerMeterPerSecond
        .of(kVVoltsPerRPM * 60 * gearing / 2 / Math.PI / drumRadius.in(Meters));
    private static final double kAVoltsPerRPMSq = 0.00014303;
    private static final Measure<Per<Voltage, Velocity<Velocity<Distance>>>> kA = VoltsPerMeterPerSecondSquared
        .of(kAVoltsPerRPMSq * 60 * gearing / 2 / Math.PI / drumRadius.in(Meters));
    private static final DCMotor gearbox = DCMotor.getNEO(1);
    private static final ElevatorFeedforward ElevatorFeedforward = new ElevatorFeedforward(
        kS.in(Volts),
        kG.in(Volts),
        kV.in(VoltsPerMeterPerSecond),
        kA.in(VoltsPerMeterPerSecondSquared));
    private static final Measure<Velocity<Distance>> maxVelocity = MetersPerSecond
        .of(ElevatorFeedforward.maxAchievableVelocity(12.0, 0.0));
    private static final Measure<Distance> minHeight = Meters.of(0.0);
    private static final Measure<Distance> maxHeight = Centimeters.of(38.5);

    private static final double velocityKPReal = 1.3129E-206;

    private static final double positionKPReal = 268.15;
    private static final double positionkDReal = 264.38;

    private static final double velocityKPSim = 1.0176E-31;

    private static final double positionKPSim = 195.52;
    private static final double positionkDSim = 150.68;

    private static final PIDController velocityPIDSim = new PIDController(velocityKPSim, 0, 0);
    private static final PIDController positionPIDSim = new PIDController(positionKPSim, 0, positionkDSim);
    private static final ExponentialProfile.Constraints constraints = ExponentialProfile.Constraints
        .fromCharacteristics(12.0, kV.in(VoltsPerMeterPerSecond), kA.in(VoltsPerMeterPerSecondSquared));
    private static final ExponentialProfile exponentialProfile = new ExponentialProfile(constraints);
  }

  private final MutableMeasure<Voltage> voltage;
  private final MutableMeasure<Distance> position;
  private final MutableMeasure<Dimensionless> velocityRatio;
  private final MutableMeasure<Current> current;

  private final CANSparkMax canSparkMax;
  private final REVElevatorSimulation revSimulation;

  public ElevatorSubsystem() {
    voltage = MutableMeasure.zero(Volts);
    position = MutableMeasure.zero(Meters);
    velocityRatio = MutableMeasure.zero(Value);
    current = MutableMeasure.zero(Amps);

    canSparkMax = new CANSparkMax(Constants.deviceId, Constants.type);
    canSparkMax.getEncoder()
        .setPositionConversionFactor(2 * Math.PI * Constants.drumRadius.in(Meters) / Constants.gearing);
    canSparkMax.getEncoder()
        .setVelocityConversionFactor(2 * Math.PI * Constants.drumRadius.in(Meters) / Constants.gearing / 60.0);
    canSparkMax.getPIDController().setP(Constants.velocityKPReal, 0);
    canSparkMax.getPIDController().setFF(Constants.kV.in(VoltsPerMeterPerSecond), 0);

    canSparkMax.getPIDController().setP(Constants.positionKPReal, 1);
    canSparkMax.getPIDController().setD(Constants.positionkDReal, 1);

    canSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
    revSimulation = new REVElevatorSimulation(
        Constants.deviceId,
        Constants.gearbox.stallTorqueNewtonMeters,
        Constants.gearbox.freeSpeedRadPerSec * 60 / 2 / Math.PI,
        Constants.gearbox,
        Constants.kS,
        Constants.kG,
        Constants.kV,
        Constants.kA,
        Constants.minHeight.in(Meters),
        Constants.maxHeight.in(Meters),
        0);
  }

  public void updateTelemetry() {
    voltage.mut_setMagnitude(canSparkMax.getAppliedOutput() * canSparkMax.getBusVoltage());
    position.mut_setMagnitude(canSparkMax.getEncoder().getPosition());
    velocityRatio.mut_setMagnitude(canSparkMax.getEncoder().getVelocity());
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
          ? Constants.kG.in(Volts) + Constants.kS.in(Volts) * Math.signum(velocityRatio)
          : Constants.kG.in(Volts);
      canSparkMax.getPIDController().setReference(
          velocityRatio * Constants.maxVelocity.in(MetersPerSecond),
          ControlType.kVelocity,
          0,
          arbFeedforward,
          ArbFFUnits.kVoltage);
    } else {
      double voltage = Constants.ElevatorFeedforward
          .calculate(velocityRatio * Constants.maxVelocity.in(MetersPerSecond), 0);
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

  public boolean atPosition(Measure<Distance> position, Measure<Distance> positionTolerance) {
    return MathUtil.isNear(
        position.in(Meters),
        this.position.in(Meters),
        positionTolerance.in(Meters));
  }

  public boolean atSpeed(double velocityRatio, double velocityRatioTolerance) {
    return MathUtil.isNear(velocityRatio, this.velocityRatio.in(Value), velocityRatioTolerance);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty(
        "Voltage",
        () -> GoatMath.round(voltage.in(Volts), 2),
        null);

    builder.addDoubleProperty(
        "Position",
        () -> GoatMath.round(position.in(Meters), 2),
        null);

    builder.addDoubleProperty(
        "Velocity",
        () -> GoatMath.round(velocityRatio.in(Percent), 2),
        null);
  }
}
