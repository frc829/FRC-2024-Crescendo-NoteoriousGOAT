package com.hardwareSims;

import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Time;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class KrakenX60FOCWithSim {

    public final Supplier<Double> supplyVoltage;
    public final Supplier<Double> positionRotations;
    public final Supplier<Double> velocityRPS;
    public final Consumer<Double> setVelocityRPS;
    public final Runnable update;

    private KrakenX60FOCWithSim(
            TalonFX talonFX,
            Runnable update) {
        this.update = update;
        this.supplyVoltage = () -> talonFX.getSupplyVoltage().getValueAsDouble();
        this.positionRotations = () -> talonFX.getPosition().getValueAsDouble();
        this.velocityRPS = () -> talonFX.getVelocity().getValueAsDouble();
        VelocityDutyCycle velocityDutyCycle = new VelocityDutyCycle(0.0);
        VelocityTorqueCurrentFOC velocityTorqueCurrentFOC = new VelocityTorqueCurrentFOC(0.0);
        TalonFXConfiguration configs = new TalonFXConfiguration();
        configs.Voltage.PeakForwardVoltage = 12;
        configs.Voltage.PeakReverseVoltage = -12;
        configs.TorqueCurrent.PeakForwardTorqueCurrent = 40;
        configs.TorqueCurrent.PeakReverseTorqueCurrent = 40;
        talonFX.getConfigurator().apply(configs);

        this.setVelocityRPS = (velocityRPS) -> {
            if (RobotBase.isSimulation()) {
                velocityDutyCycle.withVelocity(velocityRPS);
                talonFX.setControl(velocityDutyCycle);
            } else {
                velocityTorqueCurrentFOC.withVelocity(velocityRPS);
                talonFX.setControl(velocityTorqueCurrentFOC);
            }
        };
        Function<Integer, Consumer<Double>> setSlotKP = (slotID) -> (kP) -> {
            if (slotID == 0) {
                configs.Slot0.kP = kP;
            } else if (slotID == 1) {
                configs.Slot1.kP = kP;
            }
            talonFX.getConfigurator().apply(configs);
        };

        Function<Integer, Consumer<Double>> setSlotKI = (slotID) -> (kI) -> {
            if (slotID == 0) {
                configs.Slot0.kI = kI;
            } else if (slotID == 1) {
                configs.Slot1.kI = kI;
            }
            talonFX.getConfigurator().apply(configs);

        };

        Function<Integer, Consumer<Double>> setSlotKD = (slotID) -> (kD) -> {
            if (slotID == 0) {
                configs.Slot0.kD = kD;
            } else if (slotID == 1) {
                configs.Slot1.kD = kD;
            }
            talonFX.getConfigurator().apply(configs);

        };

        Function<Integer, Consumer<Double>> setSlotKF = (slotID) -> (kV) -> {
            if (slotID == 0) {
                configs.Slot0.kV = kV;
            } else if (slotID == 1) {
                configs.Slot1.kV = kV;
            }
            talonFX.getConfigurator().apply(configs);
        };

        Runnable enableBrakeMode = () -> {
            configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            talonFX.getConfigurator().apply(configs);
        };

        Runnable enableInvert = () -> {
            configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
            talonFX.getConfigurator().apply(configs);
        };

    }

    public static final Function<Integer, Function<String, KrakenX60FOCWithSim>> create = (
            deviceId) -> (canbus) -> {

                TalonFX talonFX = new TalonFX(deviceId, canbus);
                TalonFXSimState talonFXSimState = talonFX.getSimState();
                DCMotorSim dcMotorSim = new DCMotorSim(
                        DCMotor.getKrakenX60Foc(1),
                        1,
                        0.001);

                MutableMeasure<Time> lastTime = MutableMeasure.ofBaseUnits(Timer.getFPGATimestamp(), Seconds);
                MutableMeasure<Time> currentTime = MutableMeasure.ofBaseUnits(Timer.getFPGATimestamp(), Seconds);

                Runnable update = () -> {
                    if (RobotBase.isSimulation()) {
                        currentTime.mut_setMagnitude(Timer.getFPGATimestamp());
                        double deltaTime = currentTime.in(Seconds) - lastTime.in(Seconds);
                        dcMotorSim.setInputVoltage(talonFXSimState.getMotorVoltage());
                        dcMotorSim.update(deltaTime);
                        talonFXSimState.setRawRotorPosition(dcMotorSim.getAngularPositionRotations());
                        talonFXSimState.setRotorVelocity(dcMotorSim.getAngularVelocityRPM() / 60.0);
                        talonFXSimState.setSupplyVoltage(12 - talonFXSimState.getSupplyCurrent() * 0.002);
                        lastTime.mut_setMagnitude(currentTime.in(Seconds));
                    }
                };

                return new KrakenX60FOCWithSim(talonFX, update);
            };

}
