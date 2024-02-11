package com.compLevel1;

import static edu.wpi.first.units.Units.Meters;

import java.util.function.BooleanSupplier;
import java.util.function.Function;

import com.compLevel0.Sensor;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;

public class PositionSwitch {

    public final BooleanSupplier isOn;
    public final Runnable update;

    private PositionSwitch(
            BooleanSupplier isOn,
            Runnable update) {
        this.isOn = isOn;
        this.update = update;
    }

    public static final Function<Sensor<Distance>, Function<Measure<Distance>, Function<Measure<Distance>, PositionSwitch>>> create = (
            distanceSensor) -> (minOn) -> (maxOn) -> {
                BooleanSupplier isOn = () -> {
                    return distanceSensor.measure.in(Meters) <= maxOn.in(Meters)
                            && distanceSensor.measure.in(Meters) >= minOn.in(Meters);
                };
                Runnable update = () -> {
                    distanceSensor.update.run();
                };
                return new PositionSwitch(isOn, update);
            };

}
