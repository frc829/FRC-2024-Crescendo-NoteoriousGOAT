package com.compLevel0;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Milliseconds;

import java.util.Optional;
import java.util.function.Function;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class FieldDetector {

    public final Supplier<Optional<Pose2d>> fieldPosition;
    public final Supplier<Optional<Measure<Time>>> latency;
    public final Runnable enable;
    public final Runnable update;

    private FieldDetector(
            Supplier<Optional<Pose2d>> fieldPosition,
            Supplier<Optional<Measure<Time>>> latency,
            Runnable enable,
            Runnable update) {
        this.fieldPosition = fieldPosition;
        this.latency = latency;
        this.enable = enable;
        this.update = update;
    }

    public static final class Limelight {
        public static Function<String, Function<Supplier<Pose2d>, FieldDetector>> createLimelight = (
                name) -> (simFieldPose) -> {

                    NetworkTable table = NetworkTableInstance.getDefault().getTable(name);
                    NetworkTableEntry validTargetSupplier = table.getEntry("tv");
                    NetworkTableEntry pipelineSupplier = table.getEntry("getpipe");
                    NetworkTableEntry botpose_wpiblueSupplier = table.getEntry("botpose_wpiblue");
                    NetworkTableEntry botpose_wpiredSupplier = table.getEntry("botpose_wpired");
                    NetworkTableEntry pipelineConsumer = table.getEntry("pipeline");

                    MutableMeasure<Distance> fieldX = MutableMeasure.zero(Meters);
                    MutableMeasure<Distance> fieldY = MutableMeasure.zero(Meters);
                    MutableMeasure<Angle> fieldYaw = MutableMeasure.zero(Degrees);
                    MutableMeasure<Time> latencyMeasure = MutableMeasure.zero(Milliseconds);

                    Supplier<Optional<Pose2d>> fieldPosition = () -> {
                        if (RobotBase.isSimulation()) {
                            return Optional.of(simFieldPose.get());
                        }
                        if (validTargetSupplier.getInteger(0) == 1 && pipelineSupplier.getInteger(0) == 1) {
                            return Optional.of(new Pose2d(fieldX.in(Meters), fieldY.in(Meters),
                                    Rotation2d.fromDegrees(fieldYaw.in(Degrees))));
                        } else {
                            return Optional.empty();
                        }
                    };

                    Supplier<Optional<Measure<Time>>> latency = () -> {
                        if (RobotBase.isSimulation()) {
                            return Optional.of(Milliseconds.of(30));
                        }
                        if (validTargetSupplier.getInteger(0) == 1 && pipelineSupplier.getInteger(0) == 1) {
                            return Optional.of(Milliseconds.of(latencyMeasure.in(Milliseconds)));
                        } else {
                            return Optional.empty();
                        }
                    };

                    Runnable enable = () -> pipelineConsumer.setNumber(1);

                    Runnable update = () -> {
                        double[] poseArray = new double[7];
                        Optional<Alliance> allianceOption = DriverStation.getAlliance();
                        if (allianceOption.isPresent()) {
                            Alliance alliance = allianceOption.get();
                            if (alliance == Alliance.Blue) {
                                poseArray = botpose_wpiblueSupplier.getDoubleArray(new double[7]);
                            } else if (alliance == Alliance.Red) {
                                poseArray = botpose_wpiredSupplier.getDoubleArray(new double[7]);
                            } else {
                                poseArray = new double[7];
                            }
                        }
                        fieldX.mut_setMagnitude(poseArray[0]);
                        fieldX.mut_setMagnitude(poseArray[1]);
                        latencyMeasure.mut_setMagnitude(poseArray[6]);
                    };

                    return new FieldDetector(
                            fieldPosition,
                            latency,
                            enable,
                            update);
                };
    }

}
