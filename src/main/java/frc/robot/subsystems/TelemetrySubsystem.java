package frc.robot.subsystems;

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
