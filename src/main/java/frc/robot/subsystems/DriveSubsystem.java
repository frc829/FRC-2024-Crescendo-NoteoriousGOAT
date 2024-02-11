package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Supplier;

import com.compLevel0.FieldDetector;
import com.compLevel0.Gyroscope;
import com.compLevel0.Motor;
import com.compLevel0.ObjectDetector;
import com.compLevel1.SwerveModule;
import com.compLevel1.Telemetry;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics.SwerveDriveWheelStates;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class DriveSubsystem extends SubsystemBase {

        private static final class Constants {
                private static final DCMotor driveDCMotor = DCMotor.getNEO(1);
                private static final List<Integer> steerDeviceIds = Arrays.asList(10, 11, 12, 13);
                private static final List<Double> steerGearings = Collections.nCopies(4, -150.0 / 7.0);
                private static final List<Integer> wheelDeviceIds = Arrays.asList(20, 21, 22, 23);
                private static final List<Double> wheelGearings = Collections.nCopies(4, 6.75);
                private static final List<Measure<Distance>> wheelRadii = Collections.nCopies(4, Inches.of(2));
                private static final List<Integer> angleSensorIds = Arrays.asList(30, 31, 32, 33);
                private static final String angleSensorCanbus = "rio";
                private static final List<Double> steerkPs = Collections.nCopies(4, 2.0);
                private static final List<Double> steerkIs = Collections.nCopies(4, 0.0);
                private static final List<Double> steerkDs = Collections.nCopies(4, 0.0);
                private static final List<Double> steerkFs = Collections.nCopies(4, 0.0);
                private static final List<Double> wheelkPs = Collections.nCopies(4, 0.11);
                private static final List<Double> wheelkIs = Collections.nCopies(4, 0.5);
                private static final List<Double> wheelkDs = Collections.nCopies(4, 0.0001);
                private static final List<Double> wheelkFs = Collections.nCopies(4, 0.12);
                private static final List<Double> wheelFOCkPs = Collections.nCopies(4, 5.0);
                private static final List<Double> wheelFOCkIs = Collections.nCopies(4, 0.1);
                private static final List<Double> wheelFOCkDs = Collections.nCopies(4, 0.001);
                private static final List<Double> wheelFOCkFs = Collections.nCopies(4, 0.0);
                private static final Measure<Distance> moduleX = Inches.of(13);
                private static final Measure<Distance> moduleY = Inches.of(13);
                private static SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
                                new Translation2d(moduleX, moduleY),
                                new Translation2d(moduleX, moduleY.negate()),
                                new Translation2d(moduleX.negate(), moduleY),
                                new Translation2d(moduleX.negate(), moduleY.negate()));
                private static final Measure<Distance> driveRadius = Inches
                                .of(Math.hypot(moduleX.in(Inches), moduleY.in(Inches)));
                private static final Measure<Velocity<Distance>> maxLinearVelocity = MetersPerSecond.of(
                                driveDCMotor.freeSpeedRadPerSec * wheelRadii.get(0).in(Meters) / wheelGearings.get(0));
                private static final Measure<Velocity<Angle>> maxAngularVelocity = RadiansPerSecond.of(
                                maxLinearVelocity.in(MetersPerSecond) / driveRadius.in(Meters));

                private static final class CentersOfRotation {
                        private static final Translation2d origin = new Translation2d();
                        private static final Translation2d frontLeft = new Translation2d(Inches.of(13), Inches.of(13));
                        private static final Translation2d frontRight = new Translation2d(Inches.of(13),
                                        Inches.of(-13));
                }

                private static final class ModuleStates {
                        private static final SwerveDriveWheelStates zeroing = new SwerveDriveWheelStates(
                                        new SwerveModuleState[] {
                                                        new SwerveModuleState(),
                                                        new SwerveModuleState(),
                                                        new SwerveModuleState(),
                                                        new SwerveModuleState(),
                                        });
                }

                private static final class PathPlannerConfigs {
                        private static final PIDConstants translationConstants = new PIDConstants(8, 0, 0);
                        private static final PIDConstants rotationConstants = new PIDConstants(
                                        8, 0, 0);
                        private static final ReplanningConfig replanningConfig = new ReplanningConfig(
                                        true,
                                        true,
                                        0.5,
                                        0.5);
                        private static final BooleanSupplier shouldFlipPath = () -> {
                                var alliance = DriverStation.getAlliance();
                                if (alliance.isPresent()) {
                                        return alliance.get() == DriverStation.Alliance.Red;
                                }
                                return false;
                        };

                        private static final HolonomicPathFollowerConfig config = new HolonomicPathFollowerConfig(
                                        translationConstants,
                                        rotationConstants,
                                        maxLinearVelocity.in(MetersPerSecond),
                                        driveRadius.in(Meters),
                                        replanningConfig,
                                        0.020);
                }

                private static final class RotateInPlace {
                        private static final double kP = 1.0;
                        private static final PIDController pidController = new PIDController(kP, 0, 0);
                }

        }

        static {
                Constants.RotateInPlace.pidController.enableContinuousInput(-0.5, 0.5);
                Constants.RotateInPlace.pidController.setTolerance(0.01);
        }

        public final Field2d field2d;
        public final Supplier<Pose2d> fieldPose;
        public final List<Measure<Voltage>> steerVoltages;
        public final List<Measure<Voltage>> wheelVoltages;
        public final List<Measure<Voltage>> angleSensorVoltages;
        public final List<Measure<Angle>> sensorAngles;
        public final Supplier<ChassisSpeeds> robotSpeeds;
        public final Supplier<ChassisSpeeds> fieldSpeeds;
        public final Measure<Velocity<Velocity<Distance>>> linearAcceleration;
        public final Measure<Velocity<Velocity<Angle>>> angularAcceleration;
        public final List<Pair<String, Supplier<Optional<Pose2d>>>> objectPositions;
        public final Runnable resetSteerEncodersFromAbsolutes;
        public final Runnable stop;
        public final Runnable update;

        public final Supplier<Command> createStopCommand;
        public final Command resetSteerEncodersCommand;
        public final Command zeroModulesCommand;
        public final Command robotCentricOriginCommand;
        public final Command fieldCentricOriginCommand;

        public final Function<Translation2d, Command> createPointToLocationCommand;

        private DriveSubsystem(
                        Field2d field2d,
                        List<Measure<Voltage>> steerVoltages,
                        List<Measure<Voltage>> wheelVoltages,
                        List<Measure<Voltage>> angleSensorVoltages,
                        List<Measure<Angle>> sensorAngles,
                        Supplier<Pose2d> fieldPose,
                        Supplier<ChassisSpeeds> fieldSpeeds,
                        Supplier<ChassisSpeeds> robotSpeeds,
                        Measure<Velocity<Velocity<Distance>>> linearAcceleration,
                        Measure<Velocity<Velocity<Angle>>> angularAcceleration,
                        Consumer<Pose2d> resetPose,
                        Consumer<SwerveDriveWheelStates> controlModules,
                        Function<Translation2d, Consumer<ChassisSpeeds>> controlRobotChassisSpeeds,
                        Function<Translation2d, Consumer<ChassisSpeeds>> controlFieldChassisSpeeds,
                        List<Pair<String, Supplier<Optional<Pose2d>>>> objectPositions,
                        Runnable resetSteerEncodersFromAbsolutes,
                        Runnable stop,
                        Runnable update) {
                this.field2d = field2d;
                this.fieldPose = fieldPose;
                this.steerVoltages = steerVoltages;
                this.wheelVoltages = wheelVoltages;
                this.angleSensorVoltages = angleSensorVoltages;
                this.sensorAngles = sensorAngles;
                this.robotSpeeds = robotSpeeds;
                this.fieldSpeeds = fieldSpeeds;
                this.linearAcceleration = linearAcceleration;
                this.angularAcceleration = angularAcceleration;
                this.objectPositions = objectPositions;
                this.resetSteerEncodersFromAbsolutes = resetSteerEncodersFromAbsolutes;
                this.stop = stop;
                this.update = update;

                createStopCommand = () -> {
                        Command stopCommand = run(stop);
                        stopCommand.setName("STOP");
                        return stopCommand;
                };

                resetSteerEncodersCommand = runOnce(resetSteerEncodersFromAbsolutes);
                resetSteerEncodersCommand.setName("Reset Steer Encoders");

                Runnable control = () -> controlModules.accept(Constants.ModuleStates.zeroing);
                zeroModulesCommand = run(control);
                zeroModulesCommand.setName("Zero Module");

                ChassisSpeeds setChassisSpeeds = new ChassisSpeeds();
                Runnable setRobotChassisSpeeds = () -> {
                        setChassisSpeeds.vxMetersPerSecond = RobotContainer.driver.rightYValue
                                        .getAsDouble()
                                        * Constants.maxLinearVelocity.in(MetersPerSecond);
                        setChassisSpeeds.vyMetersPerSecond = RobotContainer.driver.rightXValue
                                        .getAsDouble()
                                        * Constants.maxLinearVelocity.in(MetersPerSecond);
                        setChassisSpeeds.omegaRadiansPerSecond = RobotContainer.driver.fullTriggerValue
                                        .getAsDouble()
                                        * Constants.maxAngularVelocity.in(RadiansPerSecond);
                        controlRobotChassisSpeeds.apply(Constants.CentersOfRotation.origin)
                                        .accept(setChassisSpeeds);
                };
                robotCentricOriginCommand = run(setRobotChassisSpeeds);
                robotCentricOriginCommand.setName("Manual Origin Robot Chassis Speeds");

                Runnable setFieldChassisSpeeds = () -> {
                        setChassisSpeeds.vxMetersPerSecond = RobotContainer.driver.leftYValue.getAsDouble()
                                        * Constants.maxLinearVelocity.in(MetersPerSecond);
                        setChassisSpeeds.vyMetersPerSecond = RobotContainer.driver.leftXValue.getAsDouble()
                                        * Constants.maxLinearVelocity.in(MetersPerSecond);
                        setChassisSpeeds.omegaRadiansPerSecond = RobotContainer.driver.fullTriggerValue
                                        .getAsDouble()
                                        * Constants.maxAngularVelocity.in(RadiansPerSecond);
                        controlFieldChassisSpeeds.apply(Constants.CentersOfRotation.origin)
                                        .accept(setChassisSpeeds);
                };
                fieldCentricOriginCommand = run(setFieldChassisSpeeds);
                fieldCentricOriginCommand.setName("Manual Field Chassis Speeds");

                createPointToLocationCommand = (location) -> {
                        Runnable rotateInPlace = () -> {
                                Rotation2d currentRotation = fieldPose.get().getRotation();
                                Translation2d pointingVector = location.minus(fieldPose.get().getTranslation());
                                Rotation2d desiredRotation = pointingVector.getAngle();
                                double rotationsPerSecond = Constants.RotateInPlace.pidController.calculate(
                                                currentRotation.getRotations(), desiredRotation.getRotations());
                                double omegaRadiansPerSecond = Units.rotationsToRadians(rotationsPerSecond);
                                omegaRadiansPerSecond = MathUtil.clamp(omegaRadiansPerSecond,
                                                -Constants.maxAngularVelocity.in(RadiansPerSecond),
                                                Constants.maxAngularVelocity.in(RadiansPerSecond));
                                if (Constants.RotateInPlace.pidController.atSetpoint()) {
                                        omegaRadiansPerSecond = 0;
                                }
                                controlFieldChassisSpeeds.apply(new Translation2d())
                                                .accept(new ChassisSpeeds(
                                                                RobotContainer.driver.leftYValue.getAsDouble(),
                                                                RobotContainer.driver.leftXValue.getAsDouble(),
                                                                omegaRadiansPerSecond));
                                SmartDashboard.putNumber("Rotate In Place radPerSec", omegaRadiansPerSecond);

                        };
                        Command rotateInPlaceCommand = run(rotateInPlace);
                        rotateInPlaceCommand.setName("RotateInPlace");
                        return rotateInPlaceCommand;
                };

                setDefaultCommand(createStopCommand.get());

                AutoBuilder.configureHolonomic(
                                fieldPose,
                                resetPose,
                                robotSpeeds,
                                controlRobotChassisSpeeds.apply(new Translation2d()),
                                Constants.PathPlannerConfigs.config,
                                Constants.PathPlannerConfigs.shouldFlipPath,
                                this);

        }

        @Override
        public void periodic() {
                update.run();
        }

        @Override
        public void initSendable(SendableBuilder builder) {
                super.initSendable(builder);
                builder.addDoubleProperty(
                                "Max Linear Velocity (mps)",
                                () -> Constants.maxLinearVelocity.in(MetersPerSecond),
                                null);

                builder.addDoubleProperty(
                                "Max Angular Velocity (mps)",
                                () -> Constants.maxAngularVelocity.in(RadiansPerSecond),
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
        }

        public static final Supplier<DriveSubsystem> create = () -> {

                List<SwerveModule> modules = Arrays.asList(0, 1, 2, 3).stream().map(
                                (i) -> {
                                        TalonFXConfiguration config = new TalonFXConfiguration();
                                        return SwerveModule.create
                                                        .apply(Motor.REV.createCANSparkBaseNEO
                                                                        .andThen(Motor.REV.setkP.apply(1).apply(
                                                                                        Constants.steerkPs.get(
                                                                                                        i)))
                                                                        .andThen(Motor.REV.setkI.apply(1).apply(
                                                                                        Constants.steerkIs.get(
                                                                                                        i)))
                                                                        .andThen(Motor.REV.setkD.apply(1).apply(
                                                                                        Constants.steerkDs.get(
                                                                                                        i)))
                                                                        .andThen(Motor.REV.setkF.apply(1).apply(
                                                                                        Constants.steerkFs.get(
                                                                                                        i)))
                                                                        .andThen(Motor.REV.setAngleWrapping
                                                                                        .apply(
                                                                                                        Constants.steerGearings
                                                                                                                        .get(i)))
                                                                        .andThen(Motor.REV.enableBrake)
                                                                        .andThen(
                                                                                        Motor.REV.createMotorFromCANSparkBase
                                                                                                        .apply(Constants.steerGearings
                                                                                                                        .get(i)))
                                                                        .andThen(Motor.REV.setNEOMaxVelocity)
                                                                        .andThen(Motor.REV.setTurnSim
                                                                                        .apply(Constants.steerkPs
                                                                                                        .get(i))
                                                                                        .apply(Constants.steerkIs
                                                                                                        .get(i))
                                                                                        .apply(Constants.steerkDs
                                                                                                        .get(0))
                                                                                        .apply(true)
                                                                                        .apply(Constants.steerGearings
                                                                                                        .get(i)))
                                                                        .apply(Constants.steerDeviceIds.get(i)))
                                                        .apply(Motor.CTRE.createTalonFX.apply("rio")
                                                                        .andThen(Motor.CTRE.setkP.apply(0)
                                                                                        .apply(Constants.wheelkPs
                                                                                                        .get(i))
                                                                                        .apply(config))
                                                                        .andThen(Motor.CTRE.setkI.apply(0)
                                                                                        .apply(Constants.wheelkIs
                                                                                                        .get(i))
                                                                                        .apply(config))
                                                                        .andThen(Motor.CTRE.setkD.apply(0)
                                                                                        .apply(Constants.wheelkDs
                                                                                                        .get(i))
                                                                                        .apply(config))
                                                                        .andThen(Motor.CTRE.setkV.apply(0)
                                                                                        .apply(Constants.wheelkFs
                                                                                                        .get(i))
                                                                                        .apply(config))
                                                                        .andThen(Motor.CTRE.setkP.apply(2)
                                                                                        .apply(Constants.wheelFOCkPs
                                                                                                        .get(i))
                                                                                        .apply(config))
                                                                        .andThen(Motor.CTRE.setkI.apply(2)
                                                                                        .apply(Constants.wheelFOCkIs
                                                                                                        .get(i))
                                                                                        .apply(config))
                                                                        .andThen(Motor.CTRE.setkD.apply(2)
                                                                                        .apply(Constants.wheelFOCkDs
                                                                                                        .get(i))
                                                                                        .apply(config))
                                                                        .andThen(Motor.CTRE.setkV.apply(2)
                                                                                        .apply(Constants.wheelFOCkFs
                                                                                                        .get(i))
                                                                                        .apply(config))
                                                                        .andThen(Motor.CTRE.setBrake.apply(config))
                                                                        .andThen(Motor.CTRE.createMotorFromTalonFX)
                                                                        .andThen(Motor.CTRE.setKrakenX60FOCMaxVelocity)
                                                                        .apply(Constants.wheelDeviceIds.get(i)))
                                                        .apply(Constants.steerGearings.get(i))
                                                        .apply(Constants.wheelGearings.get(i))
                                                        .apply(Constants.wheelRadii.get(i))
                                                        .apply(Constants.angleSensorIds.get(i))
                                                        .apply(Constants.angleSensorCanbus);
                                }).toList();

                List<Measure<Voltage>> steerVoltages = Arrays.asList(0, 1, 2, 3).stream()
                                .map((i) -> modules.get(i).steerVoltage)
                                .toList();
                List<Measure<Voltage>> wheelVoltages = Arrays.asList(0, 1, 2, 3).stream()
                                .map((i) -> modules.get(i).steerVoltage)
                                .toList();
                List<Measure<Voltage>> angleSensorVoltages = Arrays.asList(0, 1, 2, 3).stream()
                                .map((i) -> modules.get(i).steerVoltage).toList();

                List<Measure<Angle>> sensorAngles = Arrays.asList(0, 1, 2, 3).stream()
                                .map((i) -> modules.get(i).angleFromSensor).toList();

                MutableMeasure<Velocity<Velocity<Angle>>> angularAcceleration = MutableMeasure
                                .zero(RadiansPerSecond.per(Second));

                Supplier<SwerveDriveWheelPositions> currentWheelPositions = () -> {
                        var modulePositions = Arrays.asList(0, 1, 2, 3).stream().map(
                                        (i) -> {
                                                return new SwerveModulePosition(modules.get(i).wheelPosition,
                                                                Rotation2d
                                                                                .fromDegrees(modules
                                                                                                .get(i).angle
                                                                                                .in(Degrees)));
                                        }).toList().toArray(SwerveModulePosition[]::new);
                        return new SwerveDriveWheelPositions(modulePositions);
                };
                Supplier<SwerveDriveWheelStates> currentWheelStates = () -> {
                        var moduleStates = Arrays.asList(0, 1, 2, 3).stream().map(
                                        (i) -> {
                                                return new SwerveModuleState(modules.get(i).wheelVelocity,
                                                                Rotation2d
                                                                                .fromDegrees(modules
                                                                                                .get(i).angle
                                                                                                .in(Degrees)));
                                        }).toList().toArray(SwerveModuleState[]::new);
                        return new SwerveDriveWheelStates(moduleStates);
                };
                Supplier<ChassisSpeeds> robotSpeeds = () -> {
                        return Constants.kinematics
                                        .toChassisSpeeds(currentWheelStates.get());
                };

                Consumer<SwerveDriveWheelStates> controlModuleStates = (wheelStates) -> {
                        Arrays.asList(0, 1, 2, 3).stream().forEachOrdered(
                                        (i) -> {
                                                modules.get(i).controlState.accept(wheelStates.states[i]);
                                        });
                };

                Function<Translation2d, Consumer<ChassisSpeeds>> controlRobotChassisSpeeds = (
                                centerOfRotation) -> (chassisSpeeds) -> {
                                        if (RobotBase.isSimulation()) {
                                                chassisSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.06);
                                        } else {
                                                chassisSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);
                                        }
                                        SwerveDriveWheelStates wheelSpeeds = new SwerveDriveWheelStates(
                                                        Constants.kinematics.toSwerveModuleStates(
                                                                        chassisSpeeds,
                                                                        centerOfRotation));
                                        Arrays.asList(0, 1, 2, 3).stream().forEachOrdered(
                                                        (i) -> wheelSpeeds.states[i] = SwerveModuleState
                                                                        .optimize(
                                                                                        wheelSpeeds.states[i],
                                                                                        currentWheelStates
                                                                                                        .get().states[i].angle));
                                        Arrays.asList(0, 1, 2, 3).stream().forEachOrdered(
                                                        (i) -> {
                                                                Rotation2d difference = wheelSpeeds.states[i].angle
                                                                                .minus(currentWheelStates
                                                                                                .get().states[i].angle);
                                                                double cosine = difference.getCos();
                                                                wheelSpeeds.states[i].speedMetersPerSecond *= cosine;
                                                        });
                                        controlModuleStates.accept(wheelSpeeds);

                                };

                Runnable resetSteerEncodersFromAbsolutes = () -> {
                        Arrays.asList(0, 1, 2, 3).stream().forEachOrdered(
                                        (i) -> modules.get(i).resetSteerEncoderFromAbsolute.run());
                };

                Runnable stop = () -> {
                        Arrays.asList(0, 1, 2, 3).stream().forEachOrdered(
                                        (i) -> modules.get(i).stopAndHold.run());
                };

                Telemetry telemetry = (Gyroscope.KauaiLabs.createNavxXMP)
                                .andThen(Telemetry.create)
                                .apply(robotSpeeds)
                                .apply(Constants.kinematics)
                                .apply(currentWheelPositions);

                Telemetry.addFieldDetectorToTelemetry
                                .apply(FieldDetector.Limelight.createLimelight
                                                .apply("limelightFront")
                                                .apply(telemetry.fieldPoseEstimate))
                                .apply(telemetry);

                Telemetry.addFieldDetectorToTelemetry
                                .apply(FieldDetector.Limelight.createLimelight
                                                .apply("limelightPickup")
                                                .apply(telemetry.fieldPoseEstimate))
                                .apply(telemetry);

                Pose3d cameraPosition = new Pose3d(
                                Units.inchesToMeters(-13),
                                Units.inchesToMeters(13),
                                Units.inchesToMeters(4),
                                new Rotation3d(
                                                0,
                                                0,
                                                Math.toRadians(180)));

                Telemetry.addObjectDetectorToTelemetry
                                .apply(ObjectDetector.Limelight.createLimelight
                                                .apply("limelightPickup")
                                                .apply(cameraPosition))
                                .apply(telemetry);

                Function<Translation2d, Consumer<ChassisSpeeds>> controlFieldChassisSpeeds = (
                                centerOfRotation) -> (chassisSpeeds) -> {
                                        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds,
                                                        telemetry.fieldPoseEstimate.get().getRotation());
                                        controlRobotChassisSpeeds.apply(centerOfRotation).accept(chassisSpeeds);

                                };

                Supplier<ChassisSpeeds> fieldSpeeds = () -> {
                        return ChassisSpeeds.fromRobotRelativeSpeeds(robotSpeeds.get(),
                                        telemetry.fieldPoseEstimate.get().getRotation());
                };

                Runnable update = () -> {
                        Arrays.asList(0, 1, 2, 3).stream().forEachOrdered((i) -> {
                                modules.get(i).update.run();
                        });

                        telemetry.update.run();
                        angularAcceleration.mut_setMagnitude(
                                        telemetry.accelerationMag.in(MetersPerSecondPerSecond)
                                                        / Constants.driveRadius.in(Meters));
                };

                return new DriveSubsystem(telemetry.field2d, steerVoltages, wheelVoltages, angleSensorVoltages,
                                sensorAngles, telemetry.fieldPoseEstimate, fieldSpeeds, robotSpeeds,
                                telemetry.accelerationMag, angularAcceleration, telemetry.resetFieldPosition,
                                controlModuleStates, controlRobotChassisSpeeds, controlFieldChassisSpeeds,
                                telemetry.objectDetectorOptPositions, resetSteerEncodersFromAbsolutes, stop, update);
        };
}
