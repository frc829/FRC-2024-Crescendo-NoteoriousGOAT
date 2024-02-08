package com.hardwareSims;

import static edu.wpi.first.units.Units.Seconds;

import java.util.function.Function;
import java.util.function.Supplier;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Time;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.SerialPort.Port;

public class NavxMXPWithSim {

    public final Supplier<Double> yawDegrees;
    public final Supplier<Double> worldLinearAccelXGs;
    public final Supplier<Double> worldLinearAccelYGs;
    public final Supplier<Double> worldLinearAccelZGs;
    public final Runnable update;

    private NavxMXPWithSim(
            AHRS navx,
            Runnable update) {
        this.yawDegrees = () -> (double) navx.getYaw();
        this.worldLinearAccelXGs = () -> (double) navx.getWorldLinearAccelX();
        this.worldLinearAccelYGs = () -> (double) navx.getWorldLinearAccelY();
        this.worldLinearAccelZGs = () -> (double) navx.getWorldLinearAccelZ();
        this.update = update;
    }

    public static final Function<Supplier<ChassisSpeeds>, NavxMXPWithSim> create = (
            simChassisSpeeds) -> {

        AHRS navx = new AHRS(Port.kMXP);
        int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
        SimDouble simYaw = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
        MutableMeasure<Time> lastTime = MutableMeasure.ofBaseUnits(Timer.getFPGATimestamp(), Seconds);
        MutableMeasure<Time> currentTime = MutableMeasure.ofBaseUnits(Timer.getFPGATimestamp(), Seconds);
        simYaw.set(0.0);

        Runnable update = () -> {
            if (RobotBase.isSimulation()) {
                currentTime.mut_setMagnitude(Timer.getFPGATimestamp());
                double deltaTime = currentTime.in(Seconds) - lastTime.in(Seconds);
                double yawAngularVelocity = Math.toDegrees(simChassisSpeeds.get().omegaRadiansPerSecond);
                double yaw = simYaw.get();
                double deltaYaw = yawAngularVelocity * deltaTime;
                yaw -= deltaYaw;
                yaw = MathUtil.inputModulus(yaw, -180, 180);
                simYaw.set(yaw);
                lastTime.mut_setMagnitude(currentTime.in(Seconds));
            }
        };

        return new NavxMXPWithSim(navx, update);
    };
}
