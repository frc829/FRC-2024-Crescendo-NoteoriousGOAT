package frc.robot.triggers;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;

/** Add your docs here. */
public class ComplexTriggers {

    private ComplexTriggers() {
    }

    public static final Trigger fieldCentricOriginDriveTrigger = new Trigger(RobotContainer.driver.leftX)
            .or(RobotContainer.driver.leftY);

    public static final Trigger robotCentricOriginDriveTrigger = fieldCentricOriginDriveTrigger.negate().and(
            RobotContainer.driver.rightX.or(RobotContainer.driver.rightY).or(RobotContainer.driver.fullTrigger));

    // public static final Trigger templateTrigger = new Trigger(null);

}
