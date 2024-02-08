package com.hardwareSims;

import java.util.function.Function;
import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.sim.CANcoderSimState;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.RobotBase;

public class CANcoderWithSim {

    public final Supplier<Double> supplyVoltage;
    public final Supplier<Double> absoluteAngleRotations;
    public final Runnable update;

    private CANcoderWithSim(
            CANcoder cancoder,
            Runnable update) {
        this.supplyVoltage = () -> cancoder.getSupplyVoltage().getValueAsDouble();
        this.absoluteAngleRotations = () -> cancoder.getAbsolutePosition().getValueAsDouble();
        this.update = update;
    }

    public static final Function<Integer, Function<String, Function<Measure<Angle>, CANcoderWithSim>>> create = (
            deviceId) -> (canbus) -> (simAbsoluteAngle) -> {

                CANcoder cancoder = new CANcoder(deviceId, canbus);
                CANcoderSimState cancoderSimState = cancoder.getSimState();
                cancoderSimState.setRawPosition(0.0);
                cancoder.setPosition(0.0);
                Runnable update = () -> {
                    if (RobotBase.isSimulation()) {
                        cancoderSimState.setRawPosition(simAbsoluteAngle.in(Rotations));
                    }
                };

                return new CANcoderWithSim(cancoder, update);
            };

}
