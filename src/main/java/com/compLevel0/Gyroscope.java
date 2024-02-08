package com.compLevel0;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Gs;

import java.util.function.Function;

import com.hardwareSims.NavxMXPWithSim;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;

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
        public static Function<ChassisSpeeds, NavxMXPWithSim> createNavxXMPWithSim = (simChassisSpeeds) -> {
            return NavxMXPWithSim.create.apply(simChassisSpeeds);
        };

        public static Function<NavxMXPWithSim, Gyroscope> createNavxXMP = (navxMXPWithSim) -> {
            MutableMeasure<Angle> yaw = MutableMeasure.zero(Degrees);
            MutableMeasure<Velocity<Velocity<Distance>>> accelerationX = MutableMeasure.zero(Gs);
            MutableMeasure<Velocity<Velocity<Distance>>> accelerationY = MutableMeasure.zero(Gs);
            MutableMeasure<Velocity<Velocity<Distance>>> accelerationZ = MutableMeasure.zero(Gs);

            Runnable update = () -> {
                navxMXPWithSim.update.run();
                yaw.mut_setMagnitude(-navxMXPWithSim.navx.getYaw());
                accelerationX.mut_setMagnitude(navxMXPWithSim.navx.getWorldLinearAccelX());
                accelerationY.mut_setMagnitude(navxMXPWithSim.navx.getWorldLinearAccelY());
                accelerationZ.mut_setMagnitude(navxMXPWithSim.navx.getWorldLinearAccelZ());
            };

            return new Gyroscope(yaw, accelerationX, accelerationY, accelerationZ, update);
        };
    }

}