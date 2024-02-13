package com.compLevel1;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
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
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Telemetry {

        public final Field2d field2d;
        public final Measure<Velocity<Velocity<Distance>>> accelerationMag;
        public final Supplier<Pose2d> poseEstimate;
        public final List<Pair<String, Supplier<Optional<Pose2d>>>> fieldDetectorOptPositions;
        public final List<Pair<String, Supplier<Optional<Pose2d>>>> objectDetectorOptPositions;
        public final Consumer<Pose2d> setPoseEstimate;
        public final Runnable addDetectedPoseToEstimator;
        public final Runnable resetPoseEstimateFromFieldDetectors;
        public final Runnable update;

        private Telemetry(
                        Field2d field2d,
                        Measure<Velocity<Velocity<Distance>>> accelerationMag,
                        Supplier<Pose2d> poseEstimate,
                        List<Pair<String, Supplier<Optional<Pose2d>>>> fieldDetectorOptPositions,
                        List<Pair<String, Supplier<Optional<Pose2d>>>> objectDetectorOptPositions,
                        Consumer<Pose2d> setPoseEstimate,
                        Runnable addDetectedPoseToEstimator,
                        Runnable resetPoseEstimateFromFieldDetectors,
                        Runnable update) {
                this.field2d = field2d;
                this.accelerationMag = accelerationMag;
                this.poseEstimate = poseEstimate;
                this.fieldDetectorOptPositions = fieldDetectorOptPositions;
                this.objectDetectorOptPositions = objectDetectorOptPositions;
                this.setPoseEstimate = setPoseEstimate;
                this.addDetectedPoseToEstimator = addDetectedPoseToEstimator;
                this.resetPoseEstimateFromFieldDetectors = resetPoseEstimateFromFieldDetectors;
                this.update = update;
                SmartDashboard.putData(field2d);
        }

        public static final Function<Gyroscope, Function<SwerveDriveKinematics, Function<Supplier<SwerveDriveWheelPositions>, Telemetry>>> create = (
                        gyroscope) -> (kinematics) -> (wheelPositions) -> {

                                Field2d field2d = new Field2d();
                                MutableMeasure<Velocity<Velocity<Distance>>> accelerationMag = MutableMeasure
                                                .zero(MetersPerSecondPerSecond);

                                SwerveDrivePoseEstimator swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(
                                                kinematics, Rotation2d.fromDegrees(gyroscope.yaw.in(Degrees)),
                                                wheelPositions.get().positions, new Pose2d());

                                Supplier<Pose2d> poseEstimate = () -> swerveDrivePoseEstimator.getEstimatedPosition();

                                List<FieldDetector> fieldDetectors = new ArrayList<>();
                                List<ObjectDetector> objectDetectors = new ArrayList<>();

                                List<Pair<String, Supplier<Optional<Pose2d>>>> fieldDetectorOptPositions = new ArrayList<>();
                                List<Pair<String, Supplier<Optional<Pose2d>>>> objectDetectorOptPositions = new ArrayList<>();

                                Consumer<Pose2d> setPoseEstimate = (resetPose) -> {
                                        swerveDrivePoseEstimator.resetPosition(
                                                        Rotation2d.fromDegrees(gyroscope.yaw.in(Degrees)),
                                                        wheelPositions.get().positions, resetPose);
                                };

                                Runnable addDetectedPoseToEstimator = () -> {
                                        // TODO:

                                };

                                Runnable resetPoseEstimateFromFieldDetectors = () -> {
                                        // TODO:
                                };

                                Runnable update = () -> {
                                        gyroscope.update.run();
                                        for (var fieldDetector : fieldDetectors) {
                                                fieldDetector.update.run();
                                        }
                                        for (var objectDetector : objectDetectors) {
                                                objectDetector.update.run();
                                        }
                                        field2d.setRobotPose(poseEstimate.get());
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

                                };

                                return new Telemetry(field2d, accelerationMag, poseEstimate, fieldDetectorOptPositions,
                                                objectDetectorOptPositions, setPoseEstimate,
                                                addDetectedPoseToEstimator, resetPoseEstimateFromFieldDetectors,
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
                                        Optional<Translation2d> objectDetectorTranslation = objectDetector.robotSpaceObjectTranslation
                                                        .get();
                                        if (objectDetectorTranslation.isPresent()) {
                                                Translation2d objectTranslation = objectDetectorTranslation.get();
                                                Pose2d currentFieldPosition = telemetry.poseEstimate.get();
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
