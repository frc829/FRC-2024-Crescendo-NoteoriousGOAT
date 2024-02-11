package com.compLevel0;

import java.util.function.Consumer;
import java.util.function.Function;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Time;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.RobotContainer;

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

                // #region Set Inverse and Set Brake
                public static final Function<CANSparkBase, CANSparkBase> enableBrake = (canSparkBase) -> {
                        canSparkBase.setIdleMode(IdleMode.kBrake);
                        return canSparkBase;
                };

                public static final Function<CANSparkBase, CANSparkBase> enableCoast = (canSparkBase) -> {
                        canSparkBase.setIdleMode(IdleMode.kCoast);
                        return canSparkBase;
                };

                public static final Function<CANSparkBase, CANSparkBase> invert = (canSparkBase) -> {
                        canSparkBase.setInverted(true);
                        return canSparkBase;
                };

                //// #endregion

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

        public static final class CTRE {

                // #region createTalonFX

                public static final Function<String, Function<Integer, TalonFX>> createTalonFX = (
                                canbus) -> (deviceId) -> {
                                        TalonFX talonFX = new TalonFX(deviceId, canbus);
                                        TalonFXConfiguration config = new TalonFXConfiguration();
                                        RobotContainer.orchestra.addInstrument(talonFX);
                                        // Attempt to load the chrp
                                        var status = RobotContainer.orchestra.loadMusic("rhapsody.chrp");

                                        if (!status.isOK()) {
                                                // log error
                                        } else {
                                                System.out.println(status.getDescription());
                                        }
                                        config.Voltage.PeakForwardVoltage = 12.0;
                                        config.Voltage.PeakReverseVoltage = -12.0;
                                        config.TorqueCurrent.PeakForwardTorqueCurrent = 40;
                                        config.TorqueCurrent.PeakReverseTorqueCurrent = -40;
                                        talonFX.getConfigurator().apply(config);
                                        return talonFX;
                                };
                // #endregion

                // #region pidSetters
                public static final Function<Integer, Function<Double, Function<TalonFXConfiguration, Function<TalonFX, TalonFX>>>> setkP = (
                                slotID) -> (gain) -> (talonFXConfiguration) -> (talonFX) -> {
                                        if (slotID == 0) {
                                                talonFXConfiguration.Slot0.kP = gain;
                                        } else if (slotID == 1) {
                                                talonFXConfiguration.Slot1.kP = gain;
                                        } else if (slotID == 2) {
                                                talonFXConfiguration.Slot2.kP = gain;
                                        }
                                        talonFX.getConfigurator().apply(talonFXConfiguration);
                                        return talonFX;
                                };

                public static final Function<Integer, Function<Double, Function<TalonFXConfiguration, Function<TalonFX, TalonFX>>>> setkI = (
                                slotID) -> (gain) -> (talonFXConfiguration) -> (talonFX) -> {
                                        if (slotID == 0) {
                                                talonFXConfiguration.Slot0.kI = gain;
                                        } else if (slotID == 1) {
                                                talonFXConfiguration.Slot1.kI = gain;
                                        } else if (slotID == 2) {
                                                talonFXConfiguration.Slot2.kI = gain;
                                        }
                                        talonFX.getConfigurator().apply(talonFXConfiguration);
                                        return talonFX;
                                };

                public static final Function<Integer, Function<Double, Function<TalonFXConfiguration, Function<TalonFX, TalonFX>>>> setkD = (
                                slotID) -> (gain) -> (talonFXConfiguration) -> (talonFX) -> {
                                        if (slotID == 0) {
                                                talonFXConfiguration.Slot0.kD = gain;
                                        } else if (slotID == 1) {
                                                talonFXConfiguration.Slot1.kD = gain;
                                        } else if (slotID == 2) {
                                                talonFXConfiguration.Slot2.kD = gain;
                                        }
                                        talonFX.getConfigurator().apply(talonFXConfiguration);
                                        return talonFX;
                                };

                public static final Function<Integer, Function<Double, Function<TalonFXConfiguration, Function<TalonFX, TalonFX>>>> setkV = (
                                slotID) -> (gain) -> (talonFXConfiguration) -> (talonFX) -> {
                                        if (slotID == 0) {
                                                talonFXConfiguration.Slot0.kV = gain;
                                        } else if (slotID == 1) {
                                                talonFXConfiguration.Slot1.kV = gain;
                                        } else if (slotID == 2) {
                                                talonFXConfiguration.Slot2.kV = gain;
                                        }
                                        talonFX.getConfigurator().apply(talonFXConfiguration);
                                        return talonFX;
                                };

                // #endregion

                // #region Set Inverse and Set Brake
                public static final Function<TalonFXConfiguration, Function<TalonFX, TalonFX>> setBrake = (
                                talonFXConfiguration) -> (talonFX) -> {
                                        talonFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
                                        talonFX.getConfigurator().apply(talonFXConfiguration);
                                        return talonFX;
                                };

                public static final Function<TalonFXConfiguration, Function<TalonFX, TalonFX>> setCWPositive = (
                                talonFXConfiguration) -> (talonFX) -> {
                                        talonFXConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
                                        talonFX.getConfigurator().apply(talonFXConfiguration);
                                        return talonFX;
                                };

                public static final Function<TalonFXConfiguration, Function<TalonFX, TalonFX>> setCCWPositive = (
                                talonFXConfiguration) -> (talonFX) -> {
                                        talonFXConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
                                        talonFX.getConfigurator().apply(talonFXConfiguration);
                                        return talonFX;
                                };

                // #endregion

                // #region createMotorFromTalonFX
                public static final Function<TalonFX, Motor> createMotorFromTalonFX = (talonFX) -> {

                        TalonFXSimState talonFXSimState = talonFX.getSimState();
                        DCMotorSim dcMotorSim = new DCMotorSim(
                                        DCMotor.getKrakenX60Foc(1),
                                        1,
                                        0.001);

                        MutableMeasure<Time> lastTime = MutableMeasure
                                        .ofBaseUnits(Timer.getFPGATimestamp(), Seconds);
                        MutableMeasure<Time> currentTime = MutableMeasure
                                        .ofBaseUnits(Timer.getFPGATimestamp(), Seconds);

                        MutableMeasure<Voltage> voltage = MutableMeasure.zero(Volts);
                        MutableMeasure<Angle> angle = MutableMeasure.zero(Rotations);
                        MutableMeasure<Angle> absoluteAngle = MutableMeasure.zero(Rotations);
                        MutableMeasure<Velocity<Angle>> angularVelocity = MutableMeasure
                                        .zero(RotationsPerSecond);
                        MutableMeasure<Velocity<Angle>> maxAngularVelocity = MutableMeasure
                                        .zero(RotationsPerSecond);

                        Consumer<Measure<Angle>> setRelativeEncoder = (setpoint) -> {
                        };
                        Consumer<Measure<Angle>> turn = (setpoint) -> {
                        };
                        VelocityDutyCycle velocityDutyCycle = new VelocityDutyCycle(
                                        0,
                                        0,
                                        true,
                                        0,
                                        0,
                                        false,
                                        false,
                                        false);
                        VelocityTorqueCurrentFOC velocityTorqueCurrentFOC = new VelocityTorqueCurrentFOC(
                                        0,
                                        0,
                                        0,
                                        2,
                                        false,
                                        false,
                                        false);
                        Consumer<Measure<Velocity<Angle>>> spin = (setpoint) -> {
                                if (RobotBase.isSimulation()) {
                                        velocityDutyCycle.withVelocity(setpoint.in(RotationsPerSecond));
                                        talonFX.setControl(velocityDutyCycle);
                                } else {
                                        velocityTorqueCurrentFOC.withVelocity(setpoint.in(RotationsPerSecond));
                                        talonFX.setControl(velocityTorqueCurrentFOC);
                                }
                        };
                        Consumer<Measure<Voltage>> setVoltage = (setpoint) -> {
                        };
                        NeutralOut brake = new NeutralOut();
                        Runnable stop = () -> {
                                if (RobotBase.isSimulation()) {
                                        talonFX.setControl(brake);
                                } else {
                                        talonFX.setControl(brake);
                                }
                        };
                        Runnable update = () -> {
                                if (RobotBase.isSimulation()) {
                                        currentTime.mut_setMagnitude(Timer.getFPGATimestamp());
                                        double deltaTime = currentTime.in(Seconds)
                                                        - lastTime.in(Seconds);
                                        dcMotorSim.setInputVoltage(talonFXSimState.getMotorVoltage());
                                        dcMotorSim.update(deltaTime);
                                        talonFXSimState.setRawRotorPosition(
                                                        dcMotorSim.getAngularPositionRotations());
                                        talonFXSimState.setRotorVelocity(
                                                        dcMotorSim.getAngularVelocityRPM() / 60.0);
                                        talonFXSimState.setSupplyVoltage(12
                                                        - talonFXSimState.getSupplyCurrent() * 0.002);
                                        lastTime.mut_setMagnitude(currentTime.in(Seconds));
                                }
                                voltage.mut_setMagnitude(talonFX.getSupplyVoltage().getValueAsDouble());
                                angle.mut_setMagnitude(talonFX.getPosition().getValueAsDouble());
                                angularVelocity.mut_setMagnitude(
                                                talonFX.getVelocity().getValueAsDouble());
                        };

                        return new Motor(voltage, angle, absoluteAngle, angularVelocity,
                                        maxAngularVelocity,
                                        setRelativeEncoder, turn, spin, setVoltage, stop, update);
                };
                // #endregion

                // #region setMaxVelocities
                public static final Function<Motor, Motor> setKrakenX60FOCMaxVelocity = (motor) -> {
                        return new Motor(
                                        motor.voltage,
                                        motor.angle,
                                        motor.absoluteAngle,
                                        motor.angularVelocity,
                                        RadiansPerSecond.of(DCMotor.getKrakenX60Foc(1).freeSpeedRadPerSec),
                                        motor.setRelativeEncoderAngle,
                                        motor.turn,
                                        motor.spin,
                                        motor.setVoltage,
                                        motor.stop,
                                        motor.update);
                };

                // #endregion

        };

}
