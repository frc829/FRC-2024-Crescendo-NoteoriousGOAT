package com.compLevel1;

import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import java.util.function.Consumer;
import java.util.function.Function;

import com.compLevel0.Gyroscope;
import com.types.MutablePose2d;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class Telemetry {

    public final Field2d field2d;
    public final Measure<Velocity<Velocity<Distance>>> accelerationMag;
    public final Pose2d fieldPoseEstimate;
    public final Consumer<Pose2d> resetFieldPosition;
    public final Runnable update;

    private Telemetry(
            Field2d field2d,
            Measure<Velocity<Velocity<Distance>>> accelerationMag,
            Pose2d fieldPoseEstimate,
            Consumer<Pose2d> resetFieldPosition,
            Runnable update) {
        this.field2d = field2d;
        this.accelerationMag = accelerationMag;
        this.fieldPoseEstimate = fieldPoseEstimate;
        this.resetFieldPosition = resetFieldPosition;
        this.update = update;
    }

    public static final Function<Gyroscope, Function<SwerveDriveKinematics, Function<SwerveDriveWheelPositions, Telemetry>>> create = (
            gyroscope) -> (kinematics) -> (wheelPositions) -> {

                Field2d field2d = new Field2d();
                MutableMeasure<Velocity<Velocity<Distance>>> accelerationMag = MutableMeasure
                        .zero(MetersPerSecondPerSecond);

                SwerveDrivePoseEstimator swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(
                        kinematics,
                        gyroscope.yaw,
                        wheelPositions.positions,
                        new Pose2d());

                MutablePose2d fieldPoseEstimate = new MutablePose2d();

                Consumer<Pose2d> resetFieldPosition = (resetPose) -> {
                    swerveDrivePoseEstimator.resetPosition(
                            gyroscope.yaw,
                            wheelPositions.positions,
                            resetPose);
                };

                Runnable update = () -> {
                    gyroscope.update.run();
                    field2d.setRobotPose(fieldPoseEstimate);
                    accelerationMag.mut_setMagnitude(
                            Math.pow(gyroscope.accelerationX.in(MetersPerSecondPerSecond), 2)
                                    + Math.pow(gyroscope.accelerationY.in(MetersPerSecondPerSecond), 2)
                                    + Math.pow(gyroscope.accelerationZ.in(MetersPerSecondPerSecond), 2));
                    swerveDrivePoseEstimator.update(
                            gyroscope.yaw,
                            wheelPositions.positions);
                    fieldPoseEstimate.mut_set(
                            swerveDrivePoseEstimator.getEstimatedPosition().getX(),
                            swerveDrivePoseEstimator.getEstimatedPosition().getY(),
                            swerveDrivePoseEstimator.getEstimatedPosition().getRotation().getRadians());
                };

                return new Telemetry(
                        field2d,
                        accelerationMag,
                        fieldPoseEstimate,
                        resetFieldPosition,
                        update);
            };
}
