package com.compLevel0;

import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Function;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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

    public static final class PWF {
        public static final Function<Integer, TimeOfFlight> createTimeOfFlight = (deviceId) -> new TimeOfFlight(
                deviceId);

        public static final Function<TimeOfFlight, Sensor<Distance>> createDistanceSensor = (timeOfFlight) -> {
            MutableMeasure<Voltage> voltage = MutableMeasure.ofBaseUnits(12, Volts);
            MutableMeasure<Distance> distance = MutableMeasure.zero(Millimeters);
            Runnable update = () -> {
                distance.mut_setMagnitude(timeOfFlight.getRange());
                SmartDashboard.putString("TOF" + "_Status", timeOfFlight.getStatus().toString());
            };
            return new Sensor<>(voltage, distance, update);
        };
    }

    public static final class CTRE {

        @SuppressWarnings(value = { "resource" })
        public static final Function<Integer, Function<String, Function<Measure<Angle>, Sensor<Angle>>>> createAngleSensorFromCANCoder = (
                deviceId) -> (canbus) -> (simAbsoluteAngle) -> {
                    CANcoder cancoder = new CANcoder(deviceId, canbus);
                    CANcoderSimState cancoderSimState = cancoder.getSimState();
                    cancoderSimState.setRawPosition(0.0);
                    cancoder.setPosition(0.0);

                    MutableMeasure<Voltage> voltage = MutableMeasure.zero(Volts);
                    MutableMeasure<Angle> angle = MutableMeasure.zero(Rotations);
                    Runnable update = () -> {
                        if (RobotBase.isSimulation()) {
                            cancoderSimState.setRawPosition(simAbsoluteAngle.in(Rotations));
                        }
                        voltage.mut_setMagnitude(cancoder.getSupplyVoltage().getValueAsDouble());
                        angle.mut_setMagnitude(cancoder.getAbsolutePosition().getValueAsDouble());
                    };
                    return new Sensor<>(voltage, angle, update);
                };
    }

}
