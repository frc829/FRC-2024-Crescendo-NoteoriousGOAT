package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ServoRollerSubsystem extends SubsystemBase {

  public static final class Constants {
    private static final int channel = 0;
    private static final int max = 2;
    private static final int deadbandMax = 2;
    private static final int center = 1;
    private static final int deadbandMin = 0;
    private static final int min = 0;
  }

  private final Servo servo;
  private final PWMSim servoSim;

  public ServoRollerSubsystem() {
    servo = new Servo(Constants.channel);
    servo.setBoundsMicroseconds(
        Constants.max,
        Constants.deadbandMax,
        Constants.center,
        Constants.deadbandMin,
        Constants.min);

    servoSim = new PWMSim(servo);
  }

  private double getPosition() {
    if (RobotBase.isSimulation()) {
      return servoSim.getPosition();
    } else {
      return servo.getPosition();
    }
  }

  private double getSpeed() {
    if (RobotBase.isSimulation()) {
      return servoSim.getSpeed();
    } else {
      return servo.getSpeed();
    }
  }

  private void setPosition(double position) {
    if (RobotBase.isSimulation()) {
      servoSim.setPosition(position);
    } else {
      servo.setPosition(position);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty(
        "Servo Position (us)",
        () -> getPosition(),
        null);
    builder.addDoubleProperty(
        "Servo Speed",
        () -> getSpeed(),
        null);
  }

  public Command createSetPositionCommand(double position) {
    return runOnce(() -> setPosition(position));
  }

}
