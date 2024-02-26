package com.compLevel0;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Value;

import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Dimensionless;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Time;
import edu.wpi.first.wpilibj.RobotBase;

public class FieldDetector {

    public final String name;
    public final Supplier<Optional<Pose2d>> fieldPosition;
    public final Supplier<Optional<Translation2d>> robotSpaceTargetTranslation;
    public final Supplier<Optional<Measure<Time>>> latency;
    public final Supplier<Integer> tagCount;
    public final Consumer<Integer> setPriorityTarget;
    public final Runnable enable;
    public final Runnable update;

    private FieldDetector(
            String name,
            Supplier<Optional<Pose2d>> fieldPosition,
            Supplier<Optional<Translation2d>> robotSpaceTargetTranslation,
            Supplier<Optional<Measure<Time>>> latency,
            Supplier<Integer> tagCount,
            Consumer<Integer> setPriorityTarget,
            Runnable enable,
            Runnable update) {
        this.name = name;
        this.fieldPosition = fieldPosition;
        this.tagCount = tagCount;
        this.robotSpaceTargetTranslation = robotSpaceTargetTranslation;
        this.latency = latency;
        this.setPriorityTarget = setPriorityTarget;
        this.enable = enable;
        this.update = update;
    }

    public static final class Limelight {
        public static Function<String, Function<Pose3d, Function<Supplier<Pose2d>, FieldDetector>>> createLimelight = (
                name) -> (cameraPosition) -> (simFieldPose) -> {

                    NetworkTable table = NetworkTableInstance.getDefault().getTable(name);
                    NetworkTableEntry validTargetSupplier = table.getEntry("tv");
                    NetworkTableEntry txSupplier = table.getEntry("tx");
                    NetworkTableEntry tySupplier = table.getEntry("ty");
                    NetworkTableEntry pipelineSupplier = table.getEntry("getpipe");
                    NetworkTableEntry botpose_wpiblueSupplier = table.getEntry("botpose_wpiblue");
                    NetworkTableEntry pipelineConsumer = table.getEntry("pipeline");
                    NetworkTableEntry priorityIdConsumer = table.getEntry("priorityid");

                    MutableMeasure<Distance> fieldX = MutableMeasure.zero(Meters);
                    MutableMeasure<Distance> fieldY = MutableMeasure.zero(Meters);
                    MutableMeasure<Angle> fieldYaw = MutableMeasure.zero(Degrees);
                    MutableMeasure<Time> latencyMeasure = MutableMeasure.zero(Milliseconds);
                    MutableMeasure<Dimensionless> tagCountValue = MutableMeasure.zero(Value);

                    Supplier<Double> txDegreesSupplier = () -> {
                        if (RobotBase.isSimulation()) {
                            return 25.0;
                        } else {
                            return -txSupplier.getDouble(0);
                        }
                    };

                    Supplier<Double> tyDegreesSupplier = () -> {
                        if (RobotBase.isSimulation()) {
                            return 5.0;
                        } else {
                            return tySupplier.getDouble(0);
                        }
                    };

                    Translation2d cameraTranslationRobotSpace = new Translation2d(cameraPosition.getX(),
                            cameraPosition.getY());

                    Supplier<Optional<Translation2d>> robotSpaceTargetTranslation = () -> {
                        double txDegrees = txDegreesSupplier.get();
                        double tyDegrees = tyDegreesSupplier.get();
                        double tv = RobotBase.isSimulation() ? 1.0 : validTargetSupplier.getDouble(0.0);
                        double pipeline = RobotBase.isSimulation() ? 0.0 : pipelineSupplier.getDouble(0.0);
                        if (tv == 1 && pipeline == 1) {
                            double cameraZ = cameraPosition.getZ();
                            double cameraPitch = cameraPosition.getRotation().getY();
                            double cameraYaw = cameraPosition.getRotation().getZ();
                            double distance = Math.abs(cameraZ / Math.tan(cameraPitch + Math.toRadians(tyDegrees)));
                            double objectYCamSpace = distance * Math.sin(Math.toRadians(txDegrees));
                            double objectXCamSpace = distance * Math.cos(Math.toRadians(txDegrees));
                            Translation2d objectCamSpace = new Translation2d(objectXCamSpace, objectYCamSpace);
                            Translation2d objectCamSpaceRotatedIntoRobotFrame = objectCamSpace
                                    .rotateBy(Rotation2d.fromRadians(cameraYaw));
                            Translation2d objectTranslationRobot = cameraTranslationRobotSpace
                                    .plus(objectCamSpaceRotatedIntoRobotFrame);
                            return Optional.of(objectTranslationRobot);
                        } else {
                            return Optional.empty();
                        }
                    };

                    Supplier<Optional<Pose2d>> fieldPosition = () -> {
                        if (RobotBase.isSimulation()) {
                            Pose2d simPose = simFieldPose.get();
                            return Optional.of(simPose);
                        } else {
                            if (validTargetSupplier.getInteger(0) == 1 && pipelineSupplier.getInteger(0) == 1) {
                                double[] poseArray = botpose_wpiblueSupplier.getDoubleArray(new double[11]);
                                fieldX.mut_setMagnitude(poseArray[0]);
                                fieldY.mut_setMagnitude(poseArray[1]);
                                fieldYaw.mut_setMagnitude(poseArray[5]);
                                latencyMeasure.mut_setMagnitude(poseArray[6]);
                                if (poseArray.length > 7) {
                                    tagCountValue.mut_setMagnitude(poseArray[7]);
                                }
                                return Optional.of(new Pose2d(fieldX.in(Meters), fieldY.in(Meters),
                                        Rotation2d.fromDegrees(fieldYaw.in(Degrees))));
                            } else {
                                return Optional.empty();
                            }
                        }

                    };

                    Supplier<Optional<Measure<Time>>> latency = () -> {
                        if (RobotBase.isSimulation()) {
                            return Optional.of(Milliseconds.of(0));
                        } else {
                            if (validTargetSupplier.getInteger(0) == 1 && pipelineSupplier.getInteger(0) == 1) {
                                return Optional.of(Milliseconds.of(latencyMeasure.in(Milliseconds)));
                            } else {
                                return Optional.empty();
                            }
                        }

                    };

                    Supplier<Integer> tagCount = () -> {
                        return (int) tagCountValue.in(Value);
                    };

                    Consumer<Integer> setPriorityTarget = (id) -> {
                        priorityIdConsumer.setNumber(id);
                    };

                    Runnable enable = () -> pipelineConsumer.setNumber(1);

                    Runnable update = () -> {

                    };

                    return new FieldDetector(name, fieldPosition, robotSpaceTargetTranslation, latency, tagCount, setPriorityTarget, enable, update);
                };
    }

}
