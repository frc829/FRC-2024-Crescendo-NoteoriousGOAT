package com.compLevel0;

import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Function;
import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.CANcoder;
import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotBase;

public class Sensor<T extends Unit<T>> {

    public final MutableMeasure<Voltage> voltage;
    public final MutableMeasure<T> measure;
    public final Runnable update;

    private Sensor(
            MutableMeasure<Voltage> voltage,
            MutableMeasure<T> measure,
            Runnable update) {
        this.voltage = voltage;
        this.measure = measure;
        this.update = update;
    }

    @SuppressWarnings(value = { "resource" })
    public static final class PWF {
        public static final Function<Supplier<Measure<Distance>>, Function<Integer, Sensor<Distance>>> createDistanceSensorFromTimeOfFlight = (
                simDistance) -> (deviceId) -> {
                    TimeOfFlight timeOfFlight = new TimeOfFlight(deviceId);
                    MutableMeasure<Voltage> voltage = MutableMeasure.ofBaseUnits(12, Volts);
                    MutableMeasure<Distance> distance = MutableMeasure.zero(Millimeters);
                    Runnable update = () -> {
                        if (RobotBase.isSimulation()) {
                            distance.mut_setMagnitude(simDistance.get().in(Millimeters));
                        } else {
                            distance.mut_setMagnitude(timeOfFlight.getRange());
                        }
                    };
                    return new Sensor<>(voltage, distance, update);
                };
    }

    public static final class CTRE {

        public static final Function<CANcoder, Sensor<Angle>> createAngleSensorFromCANCoder = (
                cancoder) -> {
                    MutableMeasure<Voltage> voltage = MutableMeasure.zero(Volts);
                    MutableMeasure<Angle> angle = MutableMeasure.zero(Rotations);
                    Runnable update = () -> {
                        voltage.mut_setMagnitude(cancoder.getSupplyVoltage().getValueAsDouble());
                        angle.mut_setMagnitude(cancoder.getAbsolutePosition().getValueAsDouble());
                    };
                    return new Sensor<>(voltage, angle, update);
                };
    }

}
