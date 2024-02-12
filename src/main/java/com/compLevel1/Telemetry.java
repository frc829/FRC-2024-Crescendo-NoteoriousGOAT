package com.compLevel1;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
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
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Telemetry {

        private static final class Constants {
                private static final Measure<Distance> positionErrorTolerance = Centimeters.of(5);
        }

        public final Field2d field2d;
        public final Measure<Velocity<Velocity<Distance>>> accelerationMag;
        public final Supplier<Pose2d> fieldPoseEstimate;
        public final List<FieldDetector> fieldDetectors;
        public final List<ObjectDetector> objectDetectors;
        public final List<Pair<String, Supplier<Optional<Pose2d>>>> fieldDetectorOptPositions;
        public final List<Pair<String, Supplier<Optional<Pose2d>>>> objectDetectorOptPositions;
        public final Consumer<Pose2d> resetFieldPosition;
        public final Runnable update;

        private Telemetry(
                        Field2d field2d,
                        Measure<Velocity<Velocity<Distance>>> accelerationMag,
                        Supplier<Pose2d> fieldPoseEstimate,
                        List<FieldDetector> fieldDetectors,
                        List<ObjectDetector> objectDetectors,
                        List<Pair<String, Supplier<Optional<Pose2d>>>> fieldDetectorOptPositions,
                        List<Pair<String, Supplier<Optional<Pose2d>>>> objectDetectorOptPositions,
                        Consumer<Pose2d> resetFieldPosition,
                        Runnable update) {
                this.field2d = field2d;
                this.accelerationMag = accelerationMag;
                this.fieldPoseEstimate = fieldPoseEstimate;
                this.fieldDetectors = fieldDetectors;
                this.objectDetectors = objectDetectors;
                this.fieldDetectorOptPositions = fieldDetectorOptPositions;
                this.objectDetectorOptPositions = objectDetectorOptPositions;
                this.resetFieldPosition = resetFieldPosition;
                this.update = update;
                SmartDashboard.putData(field2d);
        }

        public static final Function<Gyroscope, Function<SwerveDriveKinematics, Function<Supplier<SwerveDriveWheelPositions>, Telemetry>>> create = (
                        gyroscope) -> (kinematics) -> (wheelPositions) -> {

                                Field2d field2d = new Field2d();
                                MutableMeasure<Velocity<Velocity<Distance>>> accelerationMag = MutableMeasure
                                                .zero(MetersPerSecondPerSecond);

                                SwerveDrivePoseEstimator swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(
                                                kinematics,
                                                Rotation2d.fromDegrees(gyroscope.yaw.in(Degrees)),
                                                wheelPositions.get().positions,
                                                new Pose2d());

                                Supplier<Pose2d> fieldPoseEstimate = () -> swerveDrivePoseEstimator
                                                .getEstimatedPosition();

                                List<FieldDetector> fieldDetectors = new ArrayList<>();
                                List<ObjectDetector> objectDetectors = new ArrayList<>();

                                List<Pair<String, Supplier<Optional<Pose2d>>>> fieldDetectorOptPositions = new ArrayList<>();
                                List<Pair<String, Supplier<Optional<Pose2d>>>> objectDetectorOptPositions = new ArrayList<>();

                                Consumer<Pose2d> resetFieldPosition = (resetPose) -> {
                                        swerveDrivePoseEstimator.resetPosition(
                                                        Rotation2d.fromDegrees(gyroscope.yaw.in(Degrees)),
                                                        wheelPositions.get().positions,
                                                        resetPose);
                                };

                                Runnable update = () -> {
                                        gyroscope.update.run();
                                        for (var fieldDetector : fieldDetectors) {
                                                fieldDetector.update.run();
                                        }
                                        for (var objectDetector : objectDetectors) {
                                                objectDetector.update.run();
                                        }
                                        field2d.setRobotPose(fieldPoseEstimate.get());
                                        for (var fieldDetectorOptPosition : fieldDetectorOptPositions) {
                                                String name = fieldDetectorOptPosition.getFirst();
                                                Optional<Pose2d> position = fieldDetectorOptPosition
                                                                .getSecond().get();
                                                if (position.isPresent()) {
                                                        field2d.getObject(name + "-Pose").setPose(position.get());
                                                } else {
                                                        field2d.getObject(name + "-Pose").setPose(new Pose2d(Double.NaN,
                                                                        Double.NaN, Rotation2d.fromDegrees(0)));
                                                }
                                        }
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
                                        accelerationMag.mut_setMagnitude(
                                                        Math.pow(gyroscope.accelerationX.in(MetersPerSecondPerSecond),
                                                                        2)
                                                                        + Math.pow(gyroscope.accelerationY.in(
                                                                                        MetersPerSecondPerSecond), 2)
                                                                        + Math.pow(gyroscope.accelerationZ.in(
                                                                                        MetersPerSecondPerSecond), 2));
                                        swerveDrivePoseEstimator.update(
                                                        Rotation2d.fromDegrees(gyroscope.yaw.in(Degrees)),
                                                        wheelPositions.get().positions);

                                        for (var fieldDetector : fieldDetectors) {
                                                var pose = fieldDetector.fieldPosition.get();
                                                var latency = fieldDetector.latency.get();
                                                if (pose.isPresent() && latency.isPresent()) {
                                                        Translation2d poseEstimateTranslation = swerveDrivePoseEstimator
                                                                        .getEstimatedPosition().getTranslation();
                                                        Translation2d detectedPoseTranslation = pose.get()
                                                                        .getTranslation();
                                                        Translation2d difference = poseEstimateTranslation
                                                                        .minus(detectedPoseTranslation);
                                                        double differenceValue = difference.getNorm();
                                                        if (differenceValue <= Constants.positionErrorTolerance
                                                                        .in(Meters)) {
                                                                swerveDrivePoseEstimator.addVisionMeasurement(
                                                                                pose.get(),
                                                                                Timer.getFPGATimestamp()
                                                                                                - latency.get().in(
                                                                                                                Seconds));
                                                        } else {
                                                                swerveDrivePoseEstimator.resetPosition(
                                                                                Rotation2d.fromDegrees(gyroscope.yaw
                                                                                                .in(Degrees)),
                                                                                wheelPositions.get().positions,
                                                                                pose.get());
                                                        }

                                                }

                                        }
                                };

                                return new Telemetry(
                                                field2d,
                                                accelerationMag,
                                                fieldPoseEstimate,
                                                fieldDetectors,
                                                objectDetectors,
                                                fieldDetectorOptPositions,
                                                objectDetectorOptPositions,
                                                resetFieldPosition,
                                                update);
                        };

        public static final Function<FieldDetector, Function<Telemetry, Telemetry>> addFieldDetectorToTelemetry = (
                        fieldDetector) -> (telemetry) -> {
                                telemetry.fieldDetectorOptPositions
                                                .add(new Pair<String, Supplier<Optional<Pose2d>>>(fieldDetector.name,
                                                                fieldDetector.fieldPosition));
                                return telemetry;
                        };

        public static final Function<ObjectDetector, Function<Telemetry, Telemetry>> addObjectDetectorToTelemetry = (
                        objectDetector) -> (telemetry) -> {

                                Supplier<Optional<Pose2d>> objectDetectorOptPose = () -> {
                                        Optional<Translation2d> objectDetectorTranslation = objectDetector.objectTranslationRobotSpace
                                                        .get();
                                        if (objectDetectorTranslation.isPresent()) {
                                                Translation2d objectTranslation = objectDetectorTranslation.get();
                                                Pose2d currentFieldPosition = telemetry.fieldPoseEstimate.get();
                                                SmartDashboard.putNumberArray("objectTranslation", new double[] {
                                                                objectTranslation.getX(), objectTranslation.getY() });
                                                objectTranslation = objectTranslation.rotateBy(
                                                                currentFieldPosition.getRotation());
                                                SmartDashboard.putNumberArray("objectTranslationAfterRot",
                                                                new double[] {
                                                                                objectTranslation.getX(),
                                                                                objectTranslation.getY() });

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
                                return telemetry;
                        };

}
