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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
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
        public final Consumer<Measure<Voltage>> setVoltage;
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
                        Consumer<Measure<Voltage>> setVoltage,
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
                this.setVoltage = setVoltage;
                this.stop = stop;
                this.update = update;
        }

        public static final class REV {

                // #region createSpark's for Specific Motors
                public static final Function<Integer, CANSparkBase> createCANSparkBaseNEO = (deviceId) -> {
                        var canSparkMax = new CANSparkMax(deviceId, MotorType.kBrushless);
                        if (RobotBase.isSimulation()) {
                                REVPhysicsSim.getInstance().addSparkMax(
                                                canSparkMax,
                                                DCMotor.getNEO(1));
                        }
                        return canSparkMax;
                };

                public static final Function<Integer, CANSparkBase> createCANSparkBaseNEO550 = (deviceId) -> {
                        var canSparkMax = new CANSparkMax(deviceId, MotorType.kBrushless);
                        if (RobotBase.isSimulation()) {
                                REVPhysicsSim.getInstance().addSparkMax(
                                                canSparkMax,
                                                DCMotor.getNeo550(1));
                        }
                        return canSparkMax;
                };

                public static final Function<Integer, CANSparkBase> createCANSparkBaseNEOVortex = (deviceId) -> {
                        if (RobotBase.isSimulation()) {
                                var canSparkMax = new CANSparkMax(deviceId, MotorType.kBrushless);
                                REVPhysicsSim.getInstance().addSparkMax(
                                                canSparkMax,
                                                DCMotor.getNeoVortex(1));
                                return canSparkMax;

                        } else {
                                return new CANSparkFlex(deviceId, MotorType.kBrushless);
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

                public static final Function<Double, Function<CANSparkBase, CANSparkBase>> setAngleWrapping = (
                                gearing) -> (canSparkBase) -> {
                                        canSparkBase.getPIDController().setPositionPIDWrappingEnabled(true);
                                        canSparkBase.getPIDController().setPositionPIDWrappingMinInput(-0.5 * gearing);
                                        canSparkBase.getPIDController().setPositionPIDWrappingMaxInput(0.5 * gearing);
                                        return canSparkBase;
                                };
                // #endregion

                // #region absoluteEncoder Setters
                public static final Function<Double, Function<CANSparkBase, CANSparkBase>> setAbsolutEencoderScaleFactor = (
                                factor) -> (canSparkBase) -> {
                                        canSparkBase.getAbsoluteEncoder(Type.kDutyCycle)
                                                        .setPositionConversionFactor(factor);
                                        return canSparkBase;
                                };

                public static final Function<CANSparkBase, CANSparkBase> setInvertAbsoluteEncoder = (canSparkBase) -> {
                        canSparkBase.getAbsoluteEncoder(Type.kDutyCycle).setInverted(true);
                        return canSparkBase;
                };

                public static final Function<Measure<Angle>, Function<CANSparkBase, CANSparkBase>> setAbsoluteEncoderOffset = (
                                offset) -> (canSparkBase) -> {
                                        canSparkBase.getAbsoluteEncoder(Type.kDutyCycle)
                                                        .setZeroOffset(offset.in(Rotations));
                                        return canSparkBase;
                                };
                // #endregion

                // #region createMotorFromCANSparkMax
                public static final Function<Double, Function<CANSparkBase, Motor>> createMotorFromCANSparkBase = (
                                gearing) -> (canSparkBase) -> {
                                        MutableMeasure<Voltage> voltage = MutableMeasure.zero(Volts);
                                        MutableMeasure<Angle> angle = MutableMeasure.zero(Rotations);
                                        MutableMeasure<Angle> absoluteAngle = MutableMeasure.zero(Rotations);
                                        MutableMeasure<Velocity<Angle>> angularVelocity = MutableMeasure.zero(RPM);
                                        MutableMeasure<Velocity<Angle>> maxAngularVelocity = MutableMeasure.zero(RPM);
                                        Consumer<Measure<Angle>> setRelativeEncoder = (setpoint) -> canSparkBase
                                                        .getEncoder()
                                                        .setPosition(setpoint.in(Rotations));
                                        Consumer<Measure<Angle>> turn = (setpoint) -> canSparkBase.getPIDController()
                                                        .setReference(setpoint.in(Rotations), ControlType.kPosition, 1);
                                        Consumer<Measure<Velocity<Angle>>> spin = (setpoint) -> canSparkBase
                                                        .getPIDController()
                                                        .setReference(setpoint.in(RPM), ControlType.kVelocity, 0);
                                        Consumer<Measure<Voltage>> setVoltage = (setpoint) -> {
                                                canSparkBase.getPIDController().setReference(setpoint.in(Volts),
                                                                ControlType.kVoltage);
                                        };
                                        Runnable stop = () -> canSparkBase.getPIDController()
                                                        .setReference(0.0, ControlType.kVelocity);
                                        Runnable update = () -> {
                                                voltage.mut_setMagnitude(
                                                                canSparkBase.getAppliedOutput()
                                                                                * canSparkBase.getBusVoltage());
                                                angle.mut_setMagnitude(canSparkBase.getEncoder().getPosition());

                                                angularVelocity.mut_setMagnitude(
                                                                canSparkBase.getEncoder().getVelocity());
                                                if (RobotBase.isSimulation()) {
                                                        absoluteAngle.mut_setMagnitude(angle.in(Rotations) / gearing);
                                                } else {
                                                        absoluteAngle.mut_setMagnitude(
                                                                        canSparkBase.getAbsoluteEncoder(Type.kDutyCycle)
                                                                                        .getPosition());
                                                }

                                        };

                                        return new Motor(voltage, angle, absoluteAngle, angularVelocity,
                                                        maxAngularVelocity,
                                                        setRelativeEncoder, turn, spin, setVoltage, stop, update);
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
                                        motor.setVoltage,
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
                                        motor.setVoltage,
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
                                        motor.setVoltage,
                                        motor.stop,
                                        motor.update);
                };
                // #endregion

                // #region Handle REV Sim Control
                @SuppressWarnings({ "resource" })
                public static final Function<Double, Function<Double, Function<Double, Function<Boolean, Function<Double, Function<Motor, Motor>>>>>> setTurnSim = (
                                kP) -> (kI) -> (kD) -> (enableWrap) -> (gearing) -> (motor) -> {

                                        if (RobotBase.isSimulation()) {
                                                PIDController pidController = new PIDController(kP, kI, kD);
                                                if (enableWrap) {
                                                        pidController.enableContinuousInput(-0.5 * gearing,
                                                                        0.5 * gearing);
                                                }
                                                MutableMeasure<Voltage> voltageSetpoint = MutableMeasure.zero(Volts);
                                                Consumer<Measure<Angle>> turn = (setpoint) -> {
                                                        double measurement = motor.angle.in(Rotations);
                                                        double goal = setpoint.in(Rotations);
                                                        double volts = pidController.calculate(measurement, goal);
                                                        volts = MathUtil.clamp(volts, -12.0, 12.0);
                                                        voltageSetpoint.mut_setMagnitude(volts / 12.0);
                                                        motor.setVoltage.accept(voltageSetpoint);
                                                };
                                                return new Motor(
                                                                motor.voltage,
                                                                motor.angle,
                                                                motor.absoluteAngle,
                                                                motor.angularVelocity,
                                                                motor.maxAngularVelocity,
                                                                motor.setRelativeEncoderAngle,
                                                                turn,
                                                                motor.spin,
                                                                motor.setVoltage,
                                                                motor.stop,
                                                                motor.update);
                                        }
                                        return motor;
                                };

                public static final Function<Motor, Motor> setSpinSim = (
                                motor) -> {

                        if (RobotBase.isSimulation()) {
                                MutableMeasure<Velocity<Angle>> spinSetpoint = MutableMeasure.zero(RPM);
                                Consumer<Measure<Velocity<Angle>>> spin = (setpoint) -> {
                                        double rpm = setpoint.in(RPM);
                                        rpm = MathUtil.clamp(
                                                        rpm,
                                                        -motor.maxAngularVelocity.in(RPM),
                                                        motor.maxAngularVelocity.in(RPM));
                                        spinSetpoint.mut_setMagnitude(rpm);
                                        motor.spin.accept(spinSetpoint);

                                };

                                return new Motor(
                                                motor.voltage,
                                                motor.angle,
                                                motor.absoluteAngle,
                                                motor.angularVelocity,
                                                motor.maxAngularVelocity,
                                                motor.setRelativeEncoderAngle,
                                                motor.turn,
                                                spin,
                                                motor.setVoltage,
                                                motor.stop,
                                                motor.update);
                        }
                        return motor;
                };
                // #endregion
        };
        // #endregion

}
