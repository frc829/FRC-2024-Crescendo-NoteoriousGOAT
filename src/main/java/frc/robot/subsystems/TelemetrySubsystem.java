package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.compLevel0.FieldDetector;
import com.compLevel0.Gyroscope;
import com.compLevel0.ObjectDetector;
import com.compLevel1.Telemetry;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class TelemetrySubsystem extends SubsystemBase {

  private static final class Constants {
    private static final List<String> fieldDetectorNames = List.of("limelightFront", "limelightRear");
    private static final List<Pair<String, Pose3d>> objectDetectorNamesPositions = List.of(
        new Pair<>(
            "limelightFront",
            new Pose3d(
                Units.inchesToMeters(0),
                Units.inchesToMeters(0),
                Units.inchesToMeters(6),
                new Rotation3d(
                    0,
                    0,
                    Math.toRadians(0)))));
  }

  public final Measure<Velocity<Velocity<Distance>>> accelerationMag;
  public final Field2d field2d;
  public final Supplier<ChassisSpeeds> fieldSpeeds;
  public final Supplier<Pose2d> poseEstimate;
  public final List<Pair<String, Supplier<Optional<Pose2d>>>> fieldDetectorsPositions;
  public final List<Pair<String, Supplier<Optional<Pose2d>>>> objectPositions;
  public final Consumer<Pose2d> setPoseEstimator;
  public final Runnable resetPoseEstimateFromFieldDetectors;
  public final List<Runnable> enableFieldDetectors;
  public final List<Runnable> enableObjectDetectors;
  public final Runnable update;

  private TelemetrySubsystem(
      Measure<Velocity<Velocity<Distance>>> accelerationMag,
      Field2d field2d,
      Supplier<ChassisSpeeds> fieldSpeeds,
      Supplier<Pose2d> poseEstimate,
      List<Pair<String, Supplier<Optional<Pose2d>>>> fieldDetectorsPositions,
      List<Pair<String, Supplier<Optional<Pose2d>>>> objectPositions,
      Consumer<Pose2d> setPoseEstimator,
      Runnable resetPoseEstimateFromFieldDetectors,
      List<Runnable> enableFieldDetectors,
      List<Runnable> enableObjectDetectors,
      Runnable update) {
    this.accelerationMag = accelerationMag;
    this.field2d = field2d;
    this.fieldSpeeds = fieldSpeeds;
    this.poseEstimate = poseEstimate;
    this.fieldDetectorsPositions = fieldDetectorsPositions;
    this.objectPositions = objectPositions;
    this.setPoseEstimator = setPoseEstimator;
    this.resetPoseEstimateFromFieldDetectors = resetPoseEstimateFromFieldDetectors;
    this.enableFieldDetectors = enableFieldDetectors;
    this.enableObjectDetectors = enableObjectDetectors;
    this.update = update;

    enableFieldDetectors.stream().forEachOrdered((detector) -> detector.run());

  }

  @Override
  public void periodic() {
    update.run();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty(
        "Linear Acceleration Mag",
        () -> accelerationMag.in(MetersPerSecondPerSecond),
        null);

    builder.addDoubleProperty(
        "Field Forward Velocity (mps)",
        () -> fieldSpeeds.get().vxMetersPerSecond,
        null);

    builder.addDoubleProperty(
        "Field Strafe Velocity (mps)",
        () -> fieldSpeeds.get().vyMetersPerSecond,
        null);

    builder.addDoubleProperty(
        "Field Rotational Velocity (dps)",
        () -> Math.toDegrees(fieldSpeeds.get().omegaRadiansPerSecond),
        null);

    builder.addDoubleArrayProperty(
        "Pose Estimate",
        () -> new double[] {
            poseEstimate.get().getX(),
            poseEstimate.get().getY(),
            poseEstimate.get().getRotation().getDegrees()
        },
        null);
    for (var fieldDetectorPosition : fieldDetectorsPositions) {
      builder.addDoubleArrayProperty(
          "Field Pose from " + fieldDetectorPosition.getFirst(),
          () -> {
            Optional<Pose2d> pose = fieldDetectorPosition.getSecond().get();
            if (pose.isPresent()) {
              return new double[] {
                  poseEstimate.get().getX(),
                  poseEstimate.get().getY(),
                  poseEstimate.get().getRotation().getDegrees()
              };
            } else {
              return new double[] {
                  Double.NaN,
                  Double.NaN,
                  Double.NaN
              };
            }
          },
          null);
    }

    for (var objectDetectorPosition : objectPositions) {
      builder.addDoubleArrayProperty(
          "Object Pose from " + objectDetectorPosition.getFirst(),
          () -> {
            Optional<Pose2d> pose = objectDetectorPosition.getSecond().get();
            if (pose.isPresent()) {
              return new double[] {
                  poseEstimate.get().getX(),
                  poseEstimate.get().getY(),
                  poseEstimate.get().getRotation().getDegrees()
              };
            } else {
              return new double[] {
                  Double.NaN,
                  Double.NaN,
                  Double.NaN
              };
            }
          },
          null);
    }

  }

  public static final Supplier<TelemetrySubsystem> create = () -> {

    Telemetry telemetry = Telemetry.create
        .apply(Gyroscope.KauaiLabs.createNavxXMP
            .apply(RobotContainer.driveSubsystem.robotSpeeds))
        .apply(DriveSubsystem.Constants.kinematics)
        .apply(RobotContainer.driveSubsystem.swerveDriveWheelPositions);

    Constants.fieldDetectorNames.forEach(
        (name) -> Telemetry.addFieldDetectorToTelemetry
            .apply(FieldDetector.Limelight.createLimelight
                .apply(name)
                .apply(telemetry.poseEstimate))
            .apply(telemetry));

    Constants.objectDetectorNamesPositions.forEach(
        (namePosition) -> Telemetry.addObjectDetectorToTelemetry
            .apply(ObjectDetector.Limelight.createLimelight
                .apply(namePosition.getFirst())
                .apply(namePosition.getSecond()))
            .apply(telemetry));

    Supplier<ChassisSpeeds> fieldSpeeds = () -> {
      return ChassisSpeeds.fromFieldRelativeSpeeds(RobotContainer.driveSubsystem.robotSpeeds.get(),
          telemetry.poseEstimate.get().getRotation());
    };

    Runnable update = () -> {
      telemetry.update.run();
      // TODO: add detectedPoseTo Estimator
      // TODO: add auto pose reset
    };

    return new TelemetrySubsystem(
        telemetry.accelerationMag,
        telemetry.field2d,
        fieldSpeeds,
        telemetry.poseEstimate,
        telemetry.fieldDetectorOptPositions,
        telemetry.objectDetectorOptPositions,
        telemetry.setPoseEstimate,
        telemetry.resetPoseEstimateFromFieldDetectors,
        telemetry.enableFieldDetectors,
        telemetry.enableObjectDetectors,
        update);
  };
}
