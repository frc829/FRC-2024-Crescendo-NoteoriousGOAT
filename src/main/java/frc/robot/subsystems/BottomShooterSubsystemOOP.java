package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Value;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.VoltsPerRadianPerSecond;
import static edu.wpi.first.units.Units.VoltsPerRadianPerSecondSquared;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import com.utility.GoatMath;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Dimensionless;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Per;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BottomShooterSubsystemOOP extends SubsystemBase {

  private static final class Constants {
    private static final int deviceId = 26;
    private static final MotorType type = MotorType.kBrushless;
    private static final Measure<Voltage> kS = Volts.of(0.082167);
    private static final Measure<Per<Voltage, Velocity<Angle>>> kV = VoltsPerRadianPerSecond
        .of(0.01744179021344080984691197162799);
    private static final Measure<Per<Voltage, Velocity<Velocity<Angle>>>> kA = VoltsPerRadianPerSecondSquared
        .of(0.00274223966947335663529786724291);
    private static final double kP = 3.1552E-309;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kFF = 1.522083333333333333333333333333e-4;
    private static final DCMotor gearbox = DCMotor.getNeoVortex(1);
    private static final double gearing = 1.0;
  }

  private final MutableMeasure<Voltage> voltage;
  private final MutableMeasure<Dimensionless> velocity;
  private final CANSparkFlex canSparkFlex;
  private final DCMotorSim dcMotorSim;
  private final SimpleMotorFeedforward simpleMotorFeedforward;
  private final PIDController pidController;
  private final Measure<Velocity<Angle>> maxVelocity;

  private final SimDouble simVelocity;
  private final SimDouble simPosition;
  private final SimDouble simOutput;
  private final SimDouble simCurrent;

  public BottomShooterSubsystemOOP() {
    voltage = MutableMeasure.zero(Volts);
    velocity = MutableMeasure.zero(Value);
    canSparkFlex = new CANSparkFlex(Constants.deviceId, Constants.type);
    LinearSystem<N2, N1, N2> plant = LinearSystemId.createDCMotorSystem(Constants.kV.in(VoltsPerRadianPerSecond),
        Constants.kA.in(VoltsPerRadianPerSecondSquared));
    canSparkFlex.setIdleMode(IdleMode.kCoast);
    canSparkFlex.getPIDController().setP(Constants.kP);
    canSparkFlex.getPIDController().setI(Constants.kI);
    canSparkFlex.getPIDController().setD(Constants.kD);
    canSparkFlex.getPIDController().setFF(Constants.kFF);
    canSparkFlex.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
    dcMotorSim = new DCMotorSim(plant, Constants.gearbox, Constants.gearing);
    pidController = new PIDController(Constants.kP * 60 / 2 / Math.PI, Constants.kI, Constants.kD, 0.020);
    simpleMotorFeedforward = new SimpleMotorFeedforward(Constants.kS.in(Volts),
        Constants.kV.in(VoltsPerRadianPerSecond), Constants.kA.in(VoltsPerRadianPerSecondSquared));
    maxVelocity = RadiansPerSecond.of(
        simpleMotorFeedforward.maxAchievableVelocity(12.0, 0.0));

    String name = String.format("SPARK MAX [%s]", Constants.deviceId);
    SimDeviceSim simDeviceSim = new SimDeviceSim(name);
    simVelocity = simDeviceSim.getDouble("Velocity");
    simPosition = simDeviceSim.getDouble("Position");
    simOutput = simDeviceSim.getDouble("Applied Output");
    simCurrent = simDeviceSim.getDouble("Motor Current");
    SimDouble simStallTorque = simDeviceSim.getDouble("Stall Torque");
    SimDouble simFreeSpeed = simDeviceSim.getDouble("Free Speed");
    simStallTorque.set(Constants.gearbox.stallTorqueNewtonMeters);
    simFreeSpeed.set(Units.radiansPerSecondToRotationsPerMinute(Constants.gearbox.freeSpeedRadPerSec));

  }

  @Override
  public void periodic() {
    if (RobotBase.isReal()) {
      velocity.mut_setMagnitude(canSparkFlex.getEncoder().getVelocity() / maxVelocity.in(RPM));
      voltage.mut_setMagnitude(canSparkFlex.getAppliedOutput() * canSparkFlex.getBusVoltage());
    }
  }

  @Override
  public void simulationPeriodic() {
    velocity.mut_setMagnitude(dcMotorSim.getAngularVelocityRPM() / maxVelocity.in(RPM));
    simCurrent.set(dcMotorSim.getCurrentDrawAmps());
    simVelocity.set(dcMotorSim.getAngularVelocityRPM());
    simPosition.set(dcMotorSim.getAngularPositionRotations());
    simOutput.set(voltage.in(Volts) / 12.0);
    dcMotorSim.update(0.020);
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

  private void stop() {
    if (RobotBase.isReal()) {
      canSparkFlex.stopMotor();
    } else {
      voltage.mut_setMagnitude(0.0);
      dcMotorSim.setInputVoltage(0.0);
    }
  }

  private void setSpeed(double percentOfMax) {
    if (RobotBase.isReal()) {
      double rpm = percentOfMax * maxVelocity.in(RPM);
      if (rpm != 0) {
        canSparkFlex.getPIDController().setReference(
            rpm,
            ControlType.kVelocity,
            0,
            Constants.kS.in(Volts),
            ArbFFUnits.kVoltage);
      } else {
        canSparkFlex.getPIDController().setReference(
            rpm,
            ControlType.kVelocity);
      }
    } else {
      double currentVelocity = dcMotorSim.getAngularVelocityRadPerSec();
      double nextVelocity = percentOfMax * maxVelocity.in(RadiansPerSecond);
      double voltageFF = simpleMotorFeedforward.calculate(currentVelocity, nextVelocity, 0.020);
      double voltagePID = pidController.calculate(currentVelocity, nextVelocity);
      double voltageTotal = voltageFF + voltagePID;
      voltageTotal = MathUtil.clamp(voltageTotal, -12.0, 12.0);
      if (Math.abs(voltageTotal) < Constants.kS.in(Volts)) {
        voltageTotal = 0.0;
      }
      voltage.mut_setMagnitude(voltageTotal);
      dcMotorSim.setInputVoltage(voltageTotal);
    }
  }

  public boolean atTolerance(double expectedValue, double toleranceValue) {
    return MathUtil.isNear(expectedValue, velocity.in(Value), toleranceValue);
  }

  public Command createStopCommand() {
    return run(this::stop).withName("STOP");
  }

  public Command createSetSpeedCommand(DoubleSupplier percentOfMax) {
    return run(() -> setSpeed(percentOfMax.getAsDouble()))
        .withName(String.format("Run at %s%%", percentOfMax));
  }

}
