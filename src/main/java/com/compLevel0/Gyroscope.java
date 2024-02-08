package com.compLevel0;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Gs;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.Function;
import java.util.function.Supplier;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;

public class Gyroscope {

    public final Measure<Angle> yaw;
    public final Measure<Velocity<Velocity<Distance>>> accelerationX;
    public final Measure<Velocity<Velocity<Distance>>> accelerationY;
    public final Measure<Velocity<Velocity<Distance>>> accelerationZ;
    public final Runnable update;

    private Gyroscope(
            Measure<Angle> yaw,
            Measure<Velocity<Velocity<Distance>>> accelerationX,
            Measure<Velocity<Velocity<Distance>>> accelerationY,
            Measure<Velocity<Velocity<Distance>>> accelerationZ,
            Runnable update) {
        this.yaw = yaw;
        this.accelerationX = accelerationX;
        this.accelerationY = accelerationY;
        this.accelerationZ = accelerationZ;
        this.update = update;
    }

    public static final class KauaiLabs {
        public static Function<Supplier<ChassisSpeeds>, Gyroscope> createNavxXMP = (simChassisSpeeds) -> {
            MutableMeasure<Angle> yaw = MutableMeasure.zero(Degrees);
            MutableMeasure<Velocity<Velocity<Distance>>> accelerationX = MutableMeasure.zero(Gs);
            MutableMeasure<Velocity<Velocity<Distance>>> accelerationY = MutableMeasure.zero(Gs);
            MutableMeasure<Velocity<Velocity<Distance>>> accelerationZ = MutableMeasure.zero(Gs);

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
                    double yawValue = simYaw.get();
                    double deltaYaw = yawAngularVelocity * deltaTime;
                    yawValue -= deltaYaw;
                    yawValue = MathUtil.inputModulus(yawValue, -180, 180);
                    simYaw.set(yawValue);
                    lastTime.mut_setMagnitude(currentTime.in(Seconds));
                }
                yaw.mut_setMagnitude(-navx.getYaw());
                accelerationX.mut_setMagnitude(navx.getWorldLinearAccelX());
                accelerationY.mut_setMagnitude(navx.getWorldLinearAccelY());
                accelerationZ.mut_setMagnitude(navx.getWorldLinearAccelZ());
            };

            return new Gyroscope(yaw, accelerationX, accelerationY, accelerationZ, update);
        };
    }

}