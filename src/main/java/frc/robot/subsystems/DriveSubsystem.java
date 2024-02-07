package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Supplier;

import com.compLevel0.Gyroscope;
import com.compLevel0.Motor;
import com.compLevel1.SwerveModule;
import com.compLevel1.Telemetry;
import com.types.MutableRotation2d;

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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {

    private static final class Constants {
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
                1.0 / Units.radiansPerSecondToRotationsPerMinute(DCMotor.getNEO(1).freeSpeedRadPerSec));
        private static final Measure<Distance> moduleX = Inches.of(13);
        private static final Measure<Distance> moduleY = Inches.of(13);
        private static final Measure<Distance> driveRadius = Inches
                .of(Math.hypot(moduleX.in(Inches), moduleY.in(Inches)));
        private static SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
                new Translation2d(moduleX, moduleY),
                new Translation2d(moduleX, moduleY.negate()),
                new Translation2d(moduleX.negate(), moduleY),
                new Translation2d(moduleX.negate(), moduleY.negate()));

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

    private DriveSubsystem(
            List<Measure<Voltage>> steerVoltages,
            List<Measure<Voltage>> wheelVoltages,
            List<Measure<Voltage>> angleSensorVoltages,
            List<Measure<Angle>> sensorAngles,
            ChassisSpeeds robotSpeeds,
            Measure<Velocity<Velocity<Distance>>> linearAcceleration,
            Measure<Velocity<Velocity<Angle>>> angularAcceleration,
            Function<Translation2d, Consumer<ChassisSpeeds>> controlRobotChassisSpeeds,
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

    }

    @Override
    public void periodic() {
        update.run();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
    }

    public static final Supplier<DriveSubsystem> create = () -> {

        List<SwerveModule> modules = Arrays.asList(0, 1, 2, 3).stream().map(
                (i) -> {
                    return SwerveModule.create
                            .apply(Motor.REV.createCANSparkBaseNEO
                                    .andThen(Motor.REV.setkP.apply(1).apply(Constants.steerkPs.get(i)))
                                    .andThen(Motor.REV.setkI.apply(1).apply(Constants.steerkIs.get(i)))
                                    .andThen(Motor.REV.setkD.apply(1).apply(Constants.steerkDs.get(i)))
                                    .andThen(Motor.REV.setkF.apply(1).apply(Constants.steerkFs.get(i)))
                                    .andThen(Motor.REV.setAngleWrapping.apply(Constants.steerGearings.get(i)))
                                    .andThen(
                                            Motor.REV.createMotorFromCANSparkBase.apply(Constants.steerGearings.get(i)))
                                    .andThen(Motor.REV.setNEOMaxVelocity)
                                    .andThen(Motor.REV.setTurnSim
                                            .apply(Constants.steerkPs.get(i))
                                            .apply(Constants.steerkIs.get(i))
                                            .apply(Constants.steerkDs.get(0))
                                            .apply(true)
                                            .apply(Constants.steerGearings.get(i)))
                                    .apply(Constants.steerDeviceIds.get(i)))
                            .apply(Motor.REV.createCANSparkBaseNEO
                                    .andThen(Motor.REV.setkP.apply(0).apply(Constants.wheelkPs.get(i)))
                                    .andThen(Motor.REV.setkI.apply(0).apply(Constants.wheelkIs.get(i)))
                                    .andThen(Motor.REV.setkD.apply(0).apply(Constants.wheelkDs.get(i)))
                                    .andThen(Motor.REV.setkF.apply(0).apply(Constants.wheelkFs.get(i)))
                                    .andThen(
                                            Motor.REV.createMotorFromCANSparkBase.apply(Constants.steerGearings.get(i)))
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

        MutableRotation2d currentModule0Angle = new MutableRotation2d();
        MutableRotation2d currentModule1Angle = new MutableRotation2d();
        MutableRotation2d currentModule2Angle = new MutableRotation2d();
        MutableRotation2d currentModule3Angle = new MutableRotation2d();
        MutableRotation2d[] currentModuleAngles = new MutableRotation2d[] {
                currentModule0Angle,
                currentModule0Angle,
                currentModule0Angle,
                currentModule0Angle
        };
        SwerveModuleState currentModule0State = new SwerveModuleState(0, currentModule0Angle);
        SwerveModuleState currentModule1State = new SwerveModuleState(0, currentModule1Angle);
        SwerveModuleState currentModule2State = new SwerveModuleState(0, currentModule2Angle);
        SwerveModuleState currentModule3State = new SwerveModuleState(0, currentModule3Angle);
        SwerveModuleState[] currentModuleStates = new SwerveModuleState[] {
                currentModule0State,
                currentModule1State,
                currentModule2State,
                currentModule3State
        };
        SwerveModulePosition currentModule0Position = new SwerveModulePosition(0, currentModule0Angle);
        SwerveModulePosition currentModule1Position = new SwerveModulePosition(0, currentModule1Angle);
        SwerveModulePosition currentModule2Position = new SwerveModulePosition(0, currentModule2Angle);
        SwerveModulePosition currentModule3Position = new SwerveModulePosition(0, currentModule3Angle);
        SwerveModulePosition[] currentModulePositions = new SwerveModulePosition[] {
                currentModule0Position,
                currentModule1Position,
                currentModule2Position,
                currentModule3Position
        };
        ChassisSpeeds robotSpeeds = new ChassisSpeeds();

        Function<Translation2d, Consumer<ChassisSpeeds>> controlRobotChassisSpeeds = (
                centerOfRotation) -> (chassisSpeeds) -> {
                    chassisSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.20);
                    SwerveDriveWheelStates wheelSpeeds = new SwerveDriveWheelStates(
                            Constants.kinematics.toSwerveModuleStates(chassisSpeeds,
                                    centerOfRotation));
                    Arrays.asList(0, 1, 2, 3).stream().forEachOrdered(
                            (i) -> wheelSpeeds.states[i] = SwerveModuleState.optimize(wheelSpeeds.states[i],
                                    currentModuleAngles[i]));
                    Arrays.asList(0, 1, 2, 3).stream().forEachOrdered(
                            (i) -> wheelSpeeds.states[i].speedMetersPerSecond *= wheelSpeeds.states[i].angle
                                    .minus(currentModuleAngles[i]).getCos());
                    Arrays.asList(0, 1, 2, 3).stream().forEachOrdered(
                            (i) -> modules.get(i).controlState.accept(wheelSpeeds.states[i]));
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
                .apply(new SwerveDriveWheelPositions(currentModulePositions));

        Runnable update = () -> {
            Arrays.asList(0, 1, 2, 3).stream().forEachOrdered((i) -> {
                modules.get(i).update.run();
                currentModuleAngles[i].mut_set(modules.get(i).angle.in(Radians));
                currentModuleStates[i].speedMetersPerSecond = modules.get(i).wheelVelocity.in(MetersPerSecond);
                currentModulePositions[i].distanceMeters = modules.get(i).wheelPosition.in(Meters);
            });

            ChassisSpeeds currentRobotSpeeds = Constants.kinematics.toChassisSpeeds(currentModuleStates);
            robotSpeeds.vxMetersPerSecond = currentRobotSpeeds.vyMetersPerSecond;
            robotSpeeds.vyMetersPerSecond = currentRobotSpeeds.vyMetersPerSecond;
            robotSpeeds.omegaRadiansPerSecond = currentRobotSpeeds.omegaRadiansPerSecond;
            telemetry.update.run();
            angularAcceleration.mut_setMagnitude(
                    telemetry.accelerationMag.in(MetersPerSecondPerSecond) / Constants.driveRadius.in(Meters));
        };

        return new DriveSubsystem(
                steerVoltages,
                wheelVoltages,
                angleSensorVoltages,
                sensorAngles,
                robotSpeeds,
                telemetry.accelerationMag,
                angularAcceleration,
                controlRobotChassisSpeeds,
                resetSteerEncodersFromAbsolutes,
                stop,
                update);
    };
}
