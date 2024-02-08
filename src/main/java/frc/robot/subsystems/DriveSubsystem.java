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
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.Supplier;

import com.compLevel0.Gyroscope;
import com.compLevel0.Motor;
import com.compLevel1.SwerveModule;
import com.compLevel1.Telemetry;
import edu.wpi.first.math.geometry.Rotation2d;
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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
                private static final List<Double> wheelkPs = Collections.nCopies(4, 0.0);
                private static final List<Double> wheelkIs = Collections.nCopies(4, 0.0);
                private static final List<Double> wheelkDs = Collections.nCopies(4, 0.0);
                private static final List<Double> wheelkFs = Collections.nCopies(4,
                                1.0 / Units.radiansPerSecondToRotationsPerMinute(driveDCMotor.freeSpeedRadPerSec));
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

        }

        public final List<Measure<Voltage>> steerVoltages;
        public final List<Measure<Voltage>> wheelVoltages;
        public final List<Measure<Voltage>> angleSensorVoltages;
        public final List<Measure<Angle>> sensorAngles;
        public final ChassisSpeeds robotSpeeds;
        public final Measure<Velocity<Velocity<Distance>>> linearAcceleration;
        public final Measure<Velocity<Velocity<Angle>>> angularAcceleration;
        public final Runnable resetSteerEncodersFromAbsolutes;
        public final Runnable stop;
        public final Runnable update;

        public final Supplier<Command> createStopCommand;
        public final Function<SwerveDriveWheelStates, Command> createControlModulesCommand;
        public final Function<Translation2d, Function<DoubleSupplier, Function<DoubleSupplier, Function<DoubleSupplier, Command>>>> createManualRobotChassisSpeedsCommand;
        public final Function<Translation2d, Function<DoubleSupplier, Function<DoubleSupplier, Function<DoubleSupplier, Command>>>> createManualFieldChassisSpeedsCommand;

        private DriveSubsystem(
                        List<Measure<Voltage>> steerVoltages,
                        List<Measure<Voltage>> wheelVoltages,
                        List<Measure<Voltage>> angleSensorVoltages,
                        List<Measure<Angle>> sensorAngles,
                        ChassisSpeeds robotSpeeds,
                        Measure<Velocity<Velocity<Distance>>> linearAcceleration,
                        Measure<Velocity<Velocity<Angle>>> angularAcceleration,
                        Consumer<SwerveDriveWheelStates> controlModules,
                        Function<Translation2d, Consumer<ChassisSpeeds>> controlRobotChassisSpeeds,
                        Function<Translation2d, Consumer<ChassisSpeeds>> controlFieldChassisSpeeds,
                        Runnable resetSteerEncodersFromAbsolutes,
                        Runnable stop,
                        Runnable update) {
                this.steerVoltages = steerVoltages;
                this.wheelVoltages = wheelVoltages;
                this.angleSensorVoltages = angleSensorVoltages;
                this.sensorAngles = sensorAngles;
                this.robotSpeeds = robotSpeeds;
                this.linearAcceleration = linearAcceleration;
                this.angularAcceleration = angularAcceleration;
                this.resetSteerEncodersFromAbsolutes = resetSteerEncodersFromAbsolutes;
                this.stop = stop;
                this.update = update;

                createStopCommand = () -> {
                        Command stopCommand = run(stop);
                        stopCommand.setName("STOP");
                        return stopCommand;
                };

                createControlModulesCommand = (states) -> {
                        Runnable control = () -> controlModules.accept(states);
                        Command controlCommand = run(control);
                        controlCommand.setName("Direct Module Control");
                        return controlCommand;

                };

                ChassisSpeeds robotChassisSpeedsSetpoint = new ChassisSpeeds();
                createManualRobotChassisSpeedsCommand = (centerOfRotation) -> (forward) -> (strafe) -> (rotation) -> {
                        Runnable setRobotChassisSpeeds = () -> {
                                robotChassisSpeedsSetpoint.vxMetersPerSecond = forward.getAsDouble()
                                                * Constants.maxLinearVelocity.in(MetersPerSecond);
                                robotChassisSpeedsSetpoint.vyMetersPerSecond = strafe.getAsDouble()
                                                * Constants.maxLinearVelocity.in(MetersPerSecond);
                                robotChassisSpeedsSetpoint.omegaRadiansPerSecond = rotation.getAsDouble()
                                                * Constants.maxAngularVelocity.in(RadiansPerSecond);
                                controlRobotChassisSpeeds.apply(centerOfRotation)
                                                .accept(robotChassisSpeedsSetpoint);
                        };
                        Command setRobotChassisSpeedsCommand = run(setRobotChassisSpeeds);
                        setRobotChassisSpeedsCommand.setName("Manual Robot Chassis Speeds");
                        return setRobotChassisSpeedsCommand;
                };

                ChassisSpeeds fieldChassisSpeedsSetpoint = new ChassisSpeeds();
                createManualFieldChassisSpeedsCommand = (centerOfRotation) -> (forward) -> (strafe) -> (rotation) -> {
                        Runnable setFieldChassisSpeeds = () -> {
                                fieldChassisSpeedsSetpoint.vxMetersPerSecond = forward.getAsDouble()
                                                * Constants.maxLinearVelocity.in(MetersPerSecond);
                                fieldChassisSpeedsSetpoint.vyMetersPerSecond = strafe.getAsDouble()
                                                * Constants.maxLinearVelocity.in(MetersPerSecond);
                                fieldChassisSpeedsSetpoint.omegaRadiansPerSecond = rotation.getAsDouble()
                                                * Constants.maxAngularVelocity.in(RadiansPerSecond);
                                controlFieldChassisSpeeds.apply(centerOfRotation)
                                                .accept(fieldChassisSpeedsSetpoint);
                        };
                        Command setFieldChassisSpeedsCommand = run(setFieldChassisSpeeds);
                        setFieldChassisSpeedsCommand.setName("Manual Field Chassis Speeds");
                        return setFieldChassisSpeedsCommand;
                };

                setDefaultCommand(createStopCommand.get());

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
        }

        public static final Supplier<DriveSubsystem> create = () -> {

                List<SwerveModule> modules = Arrays.asList(0, 1, 2, 3).stream().map(
                                (i) -> {
                                        return SwerveModule.create
                                                        .apply(Motor.REV.createCANSparkBaseNEO
                                                                        .andThen(Motor.REV.setkP.apply(1).apply(
                                                                                        Constants.steerkPs.get(i)))
                                                                        .andThen(Motor.REV.setkI.apply(1).apply(
                                                                                        Constants.steerkIs.get(i)))
                                                                        .andThen(Motor.REV.setkD.apply(1).apply(
                                                                                        Constants.steerkDs.get(i)))
                                                                        .andThen(Motor.REV.setkF.apply(1).apply(
                                                                                        Constants.steerkFs.get(i)))
                                                                        .andThen(Motor.REV.setAngleWrapping.apply(
                                                                                        Constants.steerGearings.get(i)))
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
                                                        .apply(Motor.REV.createCANSparkBaseNEO
                                                                        .andThen(Motor.REV.setkP.apply(0).apply(
                                                                                        Constants.wheelkPs.get(i)))
                                                                        .andThen(Motor.REV.setkI.apply(0).apply(
                                                                                        Constants.wheelkIs.get(i)))
                                                                        .andThen(Motor.REV.setkD.apply(0).apply(
                                                                                        Constants.wheelkDs.get(i)))
                                                                        .andThen(Motor.REV.setkF.apply(0).apply(
                                                                                        Constants.wheelkFs.get(i)))
                                                                        .andThen(
                                                                                        Motor.REV.createMotorFromCANSparkBase
                                                                                                        .apply(Constants.wheelGearings
                                                                                                                        .get(i)))
                                                                        .andThen(Motor.REV.setNEOMaxVelocity)
                                                                        .andThen(Motor.REV.setSpinSim)
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
                                                return new SwerveModulePosition(modules.get(i).wheelPosition, Rotation2d
                                                                .fromDegrees(modules.get(i).angle.in(Degrees)));
                                        }).toList().toArray(SwerveModulePosition[]::new);
                        return new SwerveDriveWheelPositions(modulePositions);
                };
                Supplier<SwerveDriveWheelStates> currentWheelStates = () -> {
                        var moduleStates = Arrays.asList(0, 1, 2, 3).stream().map(
                                        (i) -> {
                                                return new SwerveModuleState(modules.get(i).wheelVelocity, Rotation2d
                                                                .fromDegrees(modules.get(i).angle.in(Degrees)));
                                        }).toList().toArray(SwerveModuleState[]::new);
                        return new SwerveDriveWheelStates(moduleStates);
                };
                ChassisSpeeds robotSpeeds = new ChassisSpeeds();

                Consumer<SwerveDriveWheelStates> controlModuleStates = (wheelStates) -> {
                        Arrays.asList(0, 1, 2, 3).stream().forEachOrdered(
                                        (i) -> {
                                                modules.get(i).controlState.accept(wheelStates.states[i]);
                                        });
                };

                Function<Translation2d, Consumer<ChassisSpeeds>> controlRobotChassisSpeeds = (
                                centerOfRotation) -> (chassisSpeeds) -> {
                                        chassisSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.020);
                                        SwerveDriveWheelStates wheelSpeeds = new SwerveDriveWheelStates(
                                                        Constants.kinematics.toSwerveModuleStates(
                                                                        chassisSpeeds,
                                                                        centerOfRotation));
                                        Arrays.asList(0, 1, 2, 3).stream().forEachOrdered(
                                                        (i) -> wheelSpeeds.states[i] = SwerveModuleState.optimize(
                                                                        wheelSpeeds.states[i],
                                                                        currentWheelStates.get().states[i].angle));
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

                Telemetry telemetry = Gyroscope.KauaiLabs.createNavxXMPWithSim
                                .andThen(Gyroscope.KauaiLabs.createNavxXMP)
                                .andThen(Telemetry.create)
                                .apply(robotSpeeds)
                                .apply(Constants.kinematics)
                                .apply(currentWheelPositions);

                Function<Translation2d, Consumer<ChassisSpeeds>> controlFieldChassisSpeeds = (
                                centerOfRotation) -> (chassisSpeeds) -> {
                                        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds,
                                                        telemetry.fieldPoseEstimate.getRotation());
                                        controlRobotChassisSpeeds.apply(centerOfRotation).accept(chassisSpeeds);

                                };

                Runnable update = () -> {
                        Arrays.asList(0, 1, 2, 3).stream().forEachOrdered((i) -> {
                                modules.get(i).update.run();
                        });

                        ChassisSpeeds currentRobotSpeeds = Constants.kinematics
                                        .toChassisSpeeds(currentWheelStates.get());
                        robotSpeeds.vxMetersPerSecond = currentRobotSpeeds.vyMetersPerSecond;
                        robotSpeeds.vyMetersPerSecond = currentRobotSpeeds.vyMetersPerSecond;
                        robotSpeeds.omegaRadiansPerSecond = currentRobotSpeeds.omegaRadiansPerSecond;
                        telemetry.update.run();
                        angularAcceleration.mut_setMagnitude(
                                        telemetry.accelerationMag.in(MetersPerSecondPerSecond)
                                                        / Constants.driveRadius.in(Meters));
                };

                return new DriveSubsystem(steerVoltages, wheelVoltages, angleSensorVoltages, sensorAngles, robotSpeeds,
                                telemetry.accelerationMag, angularAcceleration, controlModuleStates,
                                controlRobotChassisSpeeds, controlFieldChassisSpeeds, resetSteerEncodersFromAbsolutes,
                                stop, update);
        };
}
