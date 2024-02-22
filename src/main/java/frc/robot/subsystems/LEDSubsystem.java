package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {

  private static final class Constants {
    private static final int deviceId = 39;
    private static final String canbus = "CANIVORE";
    private static final double brightnessScalar = 1.0;
    private static final boolean disableWhenLOS = false;
    private static final boolean enableOptimizations = true;
    private static final boolean statusLedOffWhenActive = false;
    private static final LEDStripType stripType = LEDStripType.GRB;
    private static final boolean v5Enabled = true;
    private static final VBatOutputMode vBatOutputMode = VBatOutputMode.On;
  }

  public final Measure<Voltage> voltage;

  private LEDSubsystem(
      Measure<Voltage> voltage,
      Runnable update) {
    this.voltage = voltage;

    // Command defaultCommand = run(stop);
    // defaultCommand.setName("STOP");
    // this.setDefaultCommand(defaultCommand);

  }

  @Override
  public void periodic() {
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
  }

  public static final Supplier<LEDSubsystem> create = () -> {

    CANdle candle = new CANdle(Constants.deviceId, Constants.canbus);
    CANdleConfiguration candleConfiguration = new CANdleConfiguration();
    candleConfiguration.brightnessScalar = Constants.brightnessScalar;
    candleConfiguration.disableWhenLOS = Constants.disableWhenLOS;
    candleConfiguration.enableOptimizations = Constants.enableOptimizations;
    candleConfiguration.statusLedOffWhenActive = Constants.statusLedOffWhenActive;
    candleConfiguration.stripType = Constants.stripType;
    candleConfiguration.v5Enabled = Constants.v5Enabled;
    candleConfiguration.vBatOutputMode = Constants.vBatOutputMode;
    candle.configAllSettings(candleConfiguration);


    return new LEDSubsystem(
        null,
        () -> {});

  };
}
