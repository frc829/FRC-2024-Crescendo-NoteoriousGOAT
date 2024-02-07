package com.hardwareSims;

import static edu.wpi.first.units.Units.Seconds;

import java.util.function.Function;

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

    public final AHRS navx;
    public final Runnable update;

    private NavxMXPWithSim(
            AHRS navx,
            Runnable update) {
        this.navx = navx;
        this.update = update;
    }

    public static final Function<ChassisSpeeds, NavxMXPWithSim> create = (
            simChassisSpeeds) -> {

        AHRS navx = new AHRS(Port.kMXP);
        int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
        SimDouble simYaw = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
        MutableMeasure<Time> lastTime = MutableMeasure.ofBaseUnits(Timer.getFPGATimestamp(), Seconds);
        MutableMeasure<Time> currentTime = MutableMeasure.ofBaseUnits(Timer.getFPGATimestamp(), Seconds);

        Runnable update = () -> {
            if (RobotBase.isSimulation()) {
                currentTime.mut_setMagnitude(Timer.getFPGATimestamp());
                double deltaTime = currentTime.in(Seconds) - lastTime.in(Seconds);
                simYaw.set(0.0);
                double yawAngularVelocity = Math.toDegrees(simChassisSpeeds.omegaRadiansPerSecond);
                double yaw = simYaw.get();
                double deltaYaw = yawAngularVelocity * deltaTime;
                yaw += deltaYaw;
                yaw = MathUtil.inputModulus(yaw, -180, 180);
                simYaw.set(yaw);
                lastTime.mut_setMagnitude(currentTime.in(Seconds));

            }
        };

        return new NavxMXPWithSim(navx, update);
    };
}
