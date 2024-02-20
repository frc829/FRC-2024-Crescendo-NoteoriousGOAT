package com.compLevel1;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Value;
import java.util.function.Consumer;
import java.util.function.Function;

import com.compLevel0.Motor;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Dimensionless;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Turner {
    public final Measure<Voltage> voltage;
    public final Measure<Angle> angle;
    public final Measure<Angle> absoluteAngle;
    public final Measure<Dimensionless> velocity;
    public final Consumer<Measure<Angle>> turn;
    public final Consumer<Double> drive;
    public final Runnable resetRelEncoderFromAbsolute;
    public final Runnable hold;
    public final Runnable update;
    public final Runnable stop;

    private Turner(
            Measure<Voltage> voltage,
            Measure<Angle> angle,
            Measure<Angle> absoluteAngle,
            Measure<Dimensionless> velocity,
            Consumer<Measure<Angle>> turn,
            Consumer<Double> drive,
            Runnable resetRelEncoderFromAbsolute,
            Runnable hold,
            Runnable update,
            Runnable stop) {
        this.voltage = voltage;
        this.angle = angle;
        this.absoluteAngle = absoluteAngle;
        this.velocity = velocity;
        this.turn = turn;
        this.drive = drive;
        this.resetRelEncoderFromAbsolute = resetRelEncoderFromAbsolute;
        this.hold = hold;
        this.update = update;
        this.stop = stop;
    }

    public static final Function<Double, Function<Double, Function<Double, Function<Motor, Turner>>>> create = (
            gearing) -> (absGearingUp) -> (absGearingDown) -> (motor) -> {
                MutableMeasure<Angle> angle = MutableMeasure.zero(Rotations);
                MutableMeasure<Angle> absoluteAngle = MutableMeasure.zero(Rotations);
                MutableMeasure<Dimensionless> velocity = MutableMeasure.zero(Value);

                MutableMeasure<Angle> turnSetpoint = MutableMeasure.zero(Rotations);
                Consumer<Measure<Angle>> turn = (setpoint) -> {
                    turnSetpoint.mut_setMagnitude(setpoint.in(Rotations));
                    turnSetpoint.mut_times(gearing);
                    motor.turn.accept(turnSetpoint);
                };
                MutableMeasure<Velocity<Angle>> driveSetpoint = MutableMeasure.zero(RPM);
                Consumer<Double> drive = (setpoint) -> {
                    driveSetpoint.mut_setMagnitude(motor.maxAngularVelocity.in(RPM));
                    driveSetpoint.mut_times(setpoint);
                    turnSetpoint.mut_setMagnitude(motor.angle.in(Rotations));
                    motor.spin.accept(driveSetpoint);
                };
                Runnable resetRelEncoderFromAbsolute = () -> {
                    double absoluteAngleValue = motor.absoluteAngle.in(Rotations);
                    if (absoluteAngleValue > 0.25 * absGearingDown) {
                        absoluteAngleValue -= 1.0;
                    }
                    turnSetpoint.mut_setMagnitude(absoluteAngleValue * (absGearingUp));
                    motor.setRelativeEncoderAngle
                            .accept(turnSetpoint);

                };
                Runnable hold = () -> motor.turn.accept(turnSetpoint);
                Runnable stop = () -> motor.stop.run();
                Runnable update = () -> {
                    motor.update.run();
                    angle.mut_setMagnitude(motor.angle.in(Rotations) / gearing);
                    double absoluteAngleValue = motor.absoluteAngle.in(Rotations);
                    if (absoluteAngleValue > 0.25 * absGearingDown) {
                        absoluteAngleValue -= 1;
                    }
                    absoluteAngle.mut_setMagnitude(absoluteAngleValue / absGearingDown);
                    velocity.mut_setMagnitude(motor.angularVelocity.in(RPM));
                    velocity.mut_divide(motor.maxAngularVelocity.in(RPM));
                    SmartDashboard.putNumber("Turn Setpiont", turnSetpoint.in(Degrees) / gearing);
                };

                return new Turner(
                        motor.voltage,
                        angle,
                        absoluteAngle,
                        velocity,
                        turn,
                        drive,
                        resetRelEncoderFromAbsolute,
                        hold,
                        update,
                        stop);
            };

}
