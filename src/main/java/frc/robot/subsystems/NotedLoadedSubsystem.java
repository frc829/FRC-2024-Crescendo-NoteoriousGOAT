package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Millimeters;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.compLevel0.Sensor;
import com.compLevel1.PositionSwitch;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NotedLoadedSubsystem extends SubsystemBase {

  public static Unit<Distance> Inches;

  private static final class Constants {
    private static final int deviceId = 34;
    private static final Measure<Distance> minOn = Millimeters.of(1);
    private static final Measure<Distance> maxOn = Millimeters.of(50);
    private static final Measure<Distance> minOnBB = Millimeters.of(1);
    private static final Measure<Distance> maxOnBB = Millimeters.of(50);
  }

  public final BooleanSupplier hasNote;
  public final BooleanSupplier hasNoteBB;
  public final Runnable update;

  private NotedLoadedSubsystem(
      BooleanSupplier hasNote,
      BooleanSupplier hasNoteBB,
      Runnable update) {
    this.hasNote = hasNote;
    this.hasNoteBB = hasNoteBB;
    this.update = update;
  }

  @Override
  public void periodic() {
    update.run();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addBooleanProperty(
        "Has Note",
        hasNote,
        null);
  }

  public static final Supplier<NotedLoadedSubsystem> create = () -> {

    Supplier<Measure<Distance>> simDistance = () -> {
      // if (RobotContainer.test.a.getAsBoolean()) {
      // return Millimeters.of(5);
      // } else {
      return Millimeters.of(0);
      // }
    };

    PositionSwitch positionSwitch = Sensor.PWF.createDistanceSensorFromTimeOfFlight.apply(simDistance)
        .andThen(PositionSwitch.create)
        .apply(Constants.deviceId)
        .apply(Constants.minOn)
        .apply(Constants.maxOn)
        .apply(Constants.minOnBB)
        .apply(Constants.maxOnBB);

    return new NotedLoadedSubsystem(
        positionSwitch.isOn,
        positionSwitch.isOnBB,
        positionSwitch.update);

  };
}
