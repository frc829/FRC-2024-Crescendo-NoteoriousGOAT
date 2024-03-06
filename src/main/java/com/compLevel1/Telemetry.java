package com.compLevel1;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Seconds;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;
import java.util.function.Function;

import com.compLevel0.FieldDetector;
import com.compLevel0.Gyroscope;
import com.compLevel0.ObjectDetector;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Telemetry {

        public final Field2d field2d;
        public final Measure<Angle> gyroYaw;
        public final Measure<Velocity<Velocity<Distance>>> accelerationX;
        public final Measure<Velocity<Velocity<Distance>>> accelerationY;
        public final Supplier<Pose2d> poseEstimate;
        public final List<Pair<String, Supplier<Optional<Pose2d>>>> fieldDetectorOptPositions;
        public final List<Pair<String, Supplier<Optional<Measure<Time>>>>> fieldDetectorLatencies;
        public final List<Pair<String, Supplier<Integer>>> fieldDetectorTagCounts;
        public final List<Pair<String, Supplier<Optional<Pose2d>>>> objectDetectorOptPositions;
        public final Supplier<Optional<Double>> priorityTargetDistance;
        public final Supplier<Optional<Rotation2d>> priorityTargetRotation;
        public final Consumer<Pose2d> setPoseEstimate;
        public final Function<Pose2d, Consumer<Measure<Time>>> addDetectedPosesToEstimator;
        public final List<Consumer<Integer>> setPriorityTargetsFromFieldDetectors;
        public final List<Runnable> enableFieldDetectors;
        public final List<Runnable> enableObjectDetectors;
        public final Runnable update;

        private Telemetry(
                        Field2d field2d,
                        Measure<Angle> gyroYaw,
                        Measure<Velocity<Velocity<Distance>>> accelerationX,
                        Measure<Velocity<Velocity<Distance>>> accelerationY,
                        Supplier<Pose2d> poseEstimate,
                        List<Pair<String, Supplier<Optional<Pose2d>>>> fieldDetectorOptPositions,
                        List<Pair<String, Supplier<Optional<Measure<Time>>>>> fieldDetectorLatencies,
                        List<Pair<String, Supplier<Integer>>> fieldDetectorTagCounts,
                        List<Pair<String, Supplier<Optional<Pose2d>>>> objectDetectorOptPositions,
                        Supplier<Optional<Double>> priorityTargetDistance,
                        Supplier<Optional<Rotation2d>> priorityTargetRotation,
                        Consumer<Pose2d> setPoseEstimate,
                        Function<Pose2d, Consumer<Measure<Time>>> addDetectedPosesToEstimator,
                        List<Consumer<Integer>> setPriorityTargetsFromFieldDetectors,
                        List<Runnable> enableFieldDetectors,
                        List<Runnable> enableObjectDetectors,
                        Runnable update) {
                this.field2d = field2d;
                this.gyroYaw = gyroYaw;
                this.accelerationX = accelerationX;
                this.accelerationY = accelerationY;
                this.poseEstimate = poseEstimate;
                this.fieldDetectorOptPositions = fieldDetectorOptPositions;
                this.fieldDetectorLatencies = fieldDetectorLatencies;
                this.fieldDetectorTagCounts = fieldDetectorTagCounts;
                this.objectDetectorOptPositions = objectDetectorOptPositions;
                this.priorityTargetDistance = priorityTargetDistance;
                this.priorityTargetRotation = priorityTargetRotation;
                this.setPoseEstimate = setPoseEstimate;
                this.addDetectedPosesToEstimator = addDetectedPosesToEstimator;
                this.setPriorityTargetsFromFieldDetectors = setPriorityTargetsFromFieldDetectors;
                this.enableFieldDetectors = enableFieldDetectors;
                this.enableObjectDetectors = enableObjectDetectors;
                this.update = update;
                SmartDashboard.putData(field2d);
        }

        public static final Function<Gyroscope, Function<SwerveDriveKinematics, Function<Supplier<SwerveDriveWheelPositions>, Telemetry>>> create = (
                        gyroscope) -> (kinematics) -> (wheelPositions) -> {

                                Field2d field2d = new Field2d();
                                SwerveDrivePoseEstimator swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(
                                                kinematics,
                                                Rotation2d.fromDegrees(gyroscope.yaw.in(Degrees)),
                                                wheelPositions.get().positions,
                                                new Pose2d(),
                                                VecBuilder.fill(0.1, 0.1, 0.1),
                                                VecBuilder.fill(0.9, 0.9, 0.9));

                                Supplier<Pose2d> poseEstimate = () -> swerveDrivePoseEstimator.getEstimatedPosition();

                                List<FieldDetector> fieldDetectors = new ArrayList<>();
                                List<ObjectDetector> objectDetectors = new ArrayList<>();

                                List<Pair<String, Supplier<Optional<Pose2d>>>> fieldDetectorOptPositions = new ArrayList<>();
                                List<Pair<String, Supplier<Optional<Measure<Time>>>>> fieldDetectorOptLatencies = new ArrayList<>();
                                List<Pair<String, Supplier<Optional<Pose2d>>>> objectDetectorOptPositions = new ArrayList<>();

                                Consumer<Pose2d> setPoseEstimate = (resetPose) -> {
                                        swerveDrivePoseEstimator.resetPosition(
                                                        Rotation2d.fromDegrees(gyroscope.yaw.in(Degrees)),
                                                        wheelPositions.get().positions, resetPose);
                                };

                                Function<Pose2d, Consumer<Measure<Time>>> addDetectedPosesToEstimator = (
                                                pose) -> (latency) -> {
                                                        swerveDrivePoseEstimator.addVisionMeasurement(pose,
                                                                        Timer.getFPGATimestamp() - latency.in(Seconds));
                                                };

                                List<Consumer<Integer>> setPriorityTargetsFromFieldDetectors = new ArrayList<>();

                                List<Runnable> enableFieldDetectors = new ArrayList<>();
                                List<Runnable> enableObjectDetectors = new ArrayList<>();
                                List<Pair<String, Supplier<Integer>>> fieldDetectorTagCounts = new ArrayList<>();

                                Supplier<Optional<Double>> priorityTargetDistance = () -> {
                                        for (var fieldDetector : fieldDetectors) {
                                                var priorityTargetTranslation = fieldDetector.robotSpaceTargetTranslation
                                                                .get();
                                                if (priorityTargetTranslation.isPresent()) {
                                                        return Optional.of(priorityTargetTranslation.get().getNorm());
                                                }
                                        }
                                        return Optional.empty();
                                };

                                Supplier<Optional<Rotation2d>> priorityTargetRotation = () -> {
                                        for (var fieldDetector : fieldDetectors) {
                                                var priorityRotatOptional = fieldDetector.robotSpaceTargetHeading.get();
                                                if (priorityRotatOptional.isPresent()) {
                                                        return Optional.of(priorityRotatOptional.get());
                                                }
                                        }
                                        return Optional.empty();
                                };

                                Runnable update = () -> {
                                        gyroscope.update.run();
                                        swerveDrivePoseEstimator.updateWithTime(
                                                        Timer.getFPGATimestamp(),
                                                        Rotation2d.fromDegrees(gyroscope.yaw.in(Degrees)),
                                                        wheelPositions.get().positions);
                                        for (var fieldDetector : fieldDetectors) {
                                                fieldDetector.update.run();
                                        }
                                        for (var objectDetector : objectDetectors) {
                                                objectDetector.update.run();
                                        }
                                        field2d.setRobotPose(poseEstimate.get());
                                        for (var objectDetectorOptPosition : objectDetectorOptPositions) {
                                                String name = objectDetectorOptPosition.getFirst();
                                                Optional<Pose2d> position = objectDetectorOptPosition
                                                                .getSecond().get();
                                                if (position.isPresent()) {
                                                        field2d.getObject(name + "-ObjectPose").setPose(position.get());
                                                } else {
                                                        field2d.getObject(name + "-ObjectPose")
                                                                        .setPose(new Pose2d(Double.NaN,
                                                                                        Double.NaN,
                                                                                        Rotation2d.fromDegrees(0)));
                                                }
                                        }

                                };

                                return new Telemetry(field2d,
                                                gyroscope.yaw,
                                                gyroscope.accelerationX,
                                                gyroscope.accelerationY,
                                                poseEstimate,
                                                fieldDetectorOptPositions,
                                                fieldDetectorOptLatencies,
                                                fieldDetectorTagCounts,
                                                objectDetectorOptPositions,
                                                priorityTargetDistance,
                                                priorityTargetRotation,
                                                setPoseEstimate,
                                                addDetectedPosesToEstimator,
                                                setPriorityTargetsFromFieldDetectors,
                                                enableFieldDetectors,
                                                enableObjectDetectors, update);
                        };

        public static final Function<FieldDetector, Function<Telemetry, Telemetry>> addFieldDetectorToTelemetry = (
                        fieldDetector) -> (telemetry) -> {
                                telemetry.fieldDetectorOptPositions
                                                .add(new Pair<String, Supplier<Optional<Pose2d>>>(fieldDetector.name,
                                                                fieldDetector.fieldPosition));
                                telemetry.fieldDetectorLatencies
                                                .add(new Pair<String, Supplier<Optional<Measure<Time>>>>(
                                                                fieldDetector.name, fieldDetector.latency));

                                telemetry.fieldDetectorTagCounts.add(new Pair<String, Supplier<Integer>>(
                                                fieldDetector.name, fieldDetector.tagCount));

                                telemetry.setPriorityTargetsFromFieldDetectors.add(fieldDetector.setPriorityTarget);
                                telemetry.enableFieldDetectors.add(fieldDetector.enable);
                                fieldDetector.enable.run();
                                return telemetry;
                        };

        public static final Function<ObjectDetector, Function<Telemetry, Telemetry>> addObjectDetectorToTelemetry = (
                        objectDetector) -> (telemetry) -> {

                                Supplier<Optional<Pose2d>> objectDetectorOptPose = () -> {
                                        Optional<Translation2d> objectDetectorTranslation = objectDetector.robotSpaceObjectTranslation
                                                        .get();
                                        if (objectDetectorTranslation.isPresent()) {
                                                Translation2d objectTranslation = objectDetectorTranslation.get();
                                                Pose2d currentFieldPosition = telemetry.poseEstimate.get();
                                                objectTranslation = objectTranslation.rotateBy(
                                                                currentFieldPosition.getRotation());

                                                objectTranslation = currentFieldPosition.getTranslation()
                                                                .plus(objectTranslation);
                                                Translation2d pathTranslation = objectTranslation
                                                                .minus(currentFieldPosition.getTranslation());
                                                Rotation2d objectRotation2d = pathTranslation.getAngle();
                                                Pose2d objectPose = new Pose2d(objectTranslation, objectRotation2d);

                                                return Optional.of(objectPose);
                                        } else {
                                                return Optional.empty();
                                        }
                                };

                                telemetry.objectDetectorOptPositions
                                                .add(new Pair<String, Supplier<Optional<Pose2d>>>(objectDetector.name,
                                                                objectDetectorOptPose));
                                telemetry.enableObjectDetectors.add(objectDetector.enable);
                                return telemetry;
                        };
}
