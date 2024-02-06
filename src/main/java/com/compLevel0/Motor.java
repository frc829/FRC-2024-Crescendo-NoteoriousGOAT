package com.compLevel0;

import java.util.function.Consumer;
import java.util.function.Function;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotBase;

public class Motor {
        public final MutableMeasure<Voltage> voltage;
        public final MutableMeasure<Angle> angle;
        public final MutableMeasure<Angle> absoluteAngle;
        public final MutableMeasure<Velocity<Angle>> angularVelocity;
        public final Measure<Velocity<Angle>> maxAngularVelocity;
        public final Consumer<Measure<Angle>> setRelativeEncoderAngle;
        public final Consumer<Measure<Angle>> turn;
        public final Consumer<Measure<Velocity<Angle>>> spin;
        public final Runnable stop;
        public final Runnable update;

        private Motor(
                        MutableMeasure<Voltage> voltage,
                        MutableMeasure<Angle> angle,
                        MutableMeasure<Angle> absoluteAngle,
                        MutableMeasure<Velocity<Angle>> angularVelocity,
                        Measure<Velocity<Angle>> maxAngularVelocity,
                        Consumer<Measure<Angle>> setRelativeEncoderAngle,
                        Consumer<Measure<Angle>> turn,
                        Consumer<Measure<Velocity<Angle>>> spin,
                        Runnable stop,
                        Runnable update) {
                this.voltage = voltage;
                this.angle = angle;
                this.absoluteAngle = absoluteAngle;
                this.angularVelocity = angularVelocity;
                this.maxAngularVelocity = maxAngularVelocity;
                this.setRelativeEncoderAngle = setRelativeEncoderAngle;
                this.turn = turn;
                this.spin = spin;
                this.stop = stop;
                this.update = update;
        }

        public static final class REV {

                // #region create Motor Controllers
                public static final Function<Integer, CANSparkMax> createBrushlessCANSparkMax = (
                                deviceId) -> new CANSparkMax(
                                                deviceId, MotorType.kBrushless);

                public static final Function<Integer, CANSparkFlex> createBrushlessCANSparkFlex = (
                                deviceId) -> new CANSparkFlex(
                                                deviceId, MotorType.kBrushless);
                // #endregion

                // #region createSpark's for Specific Motors
                public static final Function<Integer, CANSparkBase> createCANSparkBaseNEO = (deviceId) -> {
                        var canSparkMax = createBrushlessCANSparkMax.apply(deviceId);
                        if (RobotBase.isSimulation()) {
                                REVPhysicsSim.getInstance().addSparkMax(
                                                canSparkMax,
                                                DCMotor.getNEO(1));
                        }
                        return canSparkMax;
                };

                public static final Function<Integer, CANSparkBase> createCANSparkBaseNEO550 = (deviceId) -> {
                        var canSparkMax = createBrushlessCANSparkMax.apply(deviceId);
                        if (RobotBase.isSimulation()) {
                                REVPhysicsSim.getInstance().addSparkMax(
                                                canSparkMax,
                                                DCMotor.getNeo550(1));
                        }
                        return canSparkMax;
                };

                public static final Function<Integer, CANSparkBase> createCANSparkBaseNEOVortex = (deviceId) -> {
                        if (RobotBase.isSimulation()) {
                                var canSparkMax = createBrushlessCANSparkMax.apply(deviceId);
                                REVPhysicsSim.getInstance().addSparkMax(
                                                canSparkMax,
                                                DCMotor.getNeoVortex(1));
                                return canSparkMax;

                        } else {
                                return createBrushlessCANSparkFlex.apply(deviceId);
                        }
                };
                // #endregion

                // #region pidSetters
                public static final Function<Integer, Function<Double, Function<CANSparkBase, CANSparkBase>>> setkP = (
                                slotID) -> (gain) -> (canSparkBase) -> {
                                        canSparkBase.getPIDController().setP(gain, slotID);
                                        return canSparkBase;
                                };

                public static final Function<Integer, Function<Double, Function<CANSparkBase, CANSparkBase>>> setkI = (
                                slotID) -> (gain) -> (canSparkBase) -> {
                                        canSparkBase.getPIDController().setI(gain, slotID);
                                        return canSparkBase;
                                };

                public static final Function<Integer, Function<Double, Function<CANSparkBase, CANSparkBase>>> setkD = (
                                slotID) -> (gain) -> (canSparkBase) -> {
                                        canSparkBase.getPIDController().setD(gain, slotID);
                                        return canSparkBase;
                                };

                public static final Function<Integer, Function<Double, Function<CANSparkBase, CANSparkBase>>> setkF = (
                                slotID) -> (gain) -> (canSparkBase) -> {
                                        canSparkBase.getPIDController().setFF(gain, slotID);
                                        return canSparkBase;
                                };
                // #endregion

                // #region createMotorFromCANSparkMax
                public static final Function<CANSparkBase, Motor> createMotorFromCANSparkBase = (canSparkBase) -> {
                        MutableMeasure<Voltage> voltage = MutableMeasure.zero(Volts);
                        MutableMeasure<Angle> angle = MutableMeasure.zero(Rotations);
                        MutableMeasure<Angle> absoluteAngle = MutableMeasure.zero(Rotations);
                        MutableMeasure<Velocity<Angle>> angularVelocity = MutableMeasure.zero(RPM);
                        MutableMeasure<Velocity<Angle>> maxAngularVelocity = MutableMeasure.zero(RPM);
                        Consumer<Measure<Angle>> setRelativeEncoder = (setpoint) -> canSparkBase.getEncoder()
                                        .setPosition(setpoint.in(Rotations));
                        Consumer<Measure<Angle>> turn = (setpoint) -> canSparkBase.getPIDController()
                                        .setReference(setpoint.in(Rotations), ControlType.kPosition, 1);
                        Consumer<Measure<Velocity<Angle>>> spin = (setpoint) -> canSparkBase.getPIDController()
                                        .setReference(setpoint.in(RPM), ControlType.kVelocity, 0);
                        Runnable stop = () -> canSparkBase.getPIDController()
                                        .setReference(0.0, ControlType.kVelocity);
                        Runnable update = () -> {
                                voltage.mut_setMagnitude(
                                                canSparkBase.getAppliedOutput() * canSparkBase.getBusVoltage());
                                angle.mut_setMagnitude(canSparkBase.getEncoder().getPosition());
                                absoluteAngle.mut_setMagnitude(
                                                canSparkBase.getAbsoluteEncoder(Type.kDutyCycle).getPosition());
                                angularVelocity.mut_setMagnitude(canSparkBase.getEncoder().getVelocity());
                        };

                        return new Motor(
                                        voltage,
                                        angle,
                                        absoluteAngle,
                                        angularVelocity,
                                        maxAngularVelocity,
                                        setRelativeEncoder,
                                        turn,
                                        spin,
                                        stop,
                                        update);
                };
                // #endregion

                // #region setMaxVelocities
                public static final Function<Motor, Motor> setNEOMaxVelocity = (motor) -> {
                        return new Motor(
                                        motor.voltage,
                                        motor.angle,
                                        motor.absoluteAngle,
                                        motor.angularVelocity,
                                        RadiansPerSecond.of(DCMotor.getNEO(1).freeSpeedRadPerSec),
                                        motor.setRelativeEncoderAngle,
                                        motor.turn,
                                        motor.spin,
                                        motor.stop,
                                        motor.update);
                };

                public static final Function<Motor, Motor> setNEO550MaxVelocity = (motor) -> {
                        return new Motor(
                                        motor.voltage,
                                        motor.angle,
                                        motor.absoluteAngle,
                                        motor.angularVelocity,
                                        RadiansPerSecond.of(DCMotor.getNeo550(1).freeSpeedRadPerSec),
                                        motor.setRelativeEncoderAngle,
                                        motor.turn,
                                        motor.spin,
                                        motor.stop,
                                        motor.update);
                };

                public static final Function<Motor, Motor> setNEOVortexMaxVelocity = (motor) -> {
                        return new Motor(
                                        motor.voltage,
                                        motor.angle,
                                        motor.absoluteAngle,
                                        motor.angularVelocity,
                                        RadiansPerSecond.of(DCMotor.getNeoVortex(1).freeSpeedRadPerSec),
                                        motor.setRelativeEncoderAngle,
                                        motor.turn,
                                        motor.spin,
                                        motor.stop,
                                        motor.update);
                };
                // #endregion

        }
}
