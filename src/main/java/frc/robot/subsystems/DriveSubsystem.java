package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Supplier;

import com.compLevel0.Motor;
import com.compLevel1.SwerveModule;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.utility.GoatMath;

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
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {

        public static final class Constants {
                private static final DCMotor driveDCMotor = DCMotor.getKrakenX60Foc(1);
                private static final List<Integer> steerDeviceIds = Arrays.asList(10, 11, 12, 13);
                private static final List<Double> steerGearings = Collections.nCopies(4, 150.0 / 7.0);
                private static final List<Integer> wheelDeviceIds = Arrays.asList(20, 21, 22, 23);
                private static final List<Double> wheelGearings = Collections.nCopies(4, -5.90);
                private static final List<Measure<Distance>> wheelRadii = Collections.nCopies(4, Inches.of(2));
                private static final List<Integer> angleSensorIds = Arrays.asList(30, 31, 32, 33);
                private static final String angleSensorCanbus = "CANIVORE";
                private static final List<Double> steerkPs = Collections.nCopies(4, 2.0);
                private static final List<Double> steerkIs = Collections.nCopies(4, 0.0);
                private static final List<Double> steerkDs = Collections.nCopies(4, 0.0);
                private static final List<Double> steerkFs = Collections.nCopies(4, 0.0);
                private static final List<Double> wheelkPs = Collections.nCopies(4, 0.0);
                private static final List<Double> wheelkIs = Collections.nCopies(4, 0.0);
                private static final List<Double> wheelkDs = Collections.nCopies(4, 0.0000);
                private static final List<Double> wheelkFs = Collections.nCopies(4, 1.0 / Units.radiansToRotations(DCMotor.getKrakenX60Foc(1).freeSpeedRadPerSec));
                private static final List<Double> wheelFOCkPs = Collections.nCopies(4, 5.0);
                private static final List<Double> wheelFOCkIs = Collections.nCopies(4, 0.00);
                private static final List<Double> wheelFOCkDs = Collections.nCopies(4, 0.000);
                private static final List<Double> wheelFOCkFs = Collections.nCopies(4, 0.0);
                private static final Measure<Distance> moduleX = Inches.of(13);
                private static final Measure<Distance> moduleY = Inches.of(13);
                public static SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
                                new Translation2d(moduleX, moduleY),
                                new Translation2d(moduleX, moduleY.negate()),
                                new Translation2d(moduleX.negate(), moduleY),
                                new Translation2d(moduleX.negate(), moduleY.negate()));
                public static final Measure<Distance> driveRadius = Inches
                                .of(Math.hypot(moduleX.in(Inches), moduleY.in(Inches)));
                public static final Measure<Velocity<Distance>> maxLinearVelocity = MetersPerSecond.of(
                                driveDCMotor.freeSpeedRadPerSec * wheelRadii.get(0).in(Meters)
                                                / Math.abs(wheelGearings.get(0)));
                public static final Measure<Velocity<Angle>> maxAngularVelocity = RadiansPerSecond.of(
                                maxLinearVelocity.in(MetersPerSecond) / driveRadius.in(Meters));

        }

        public final List<Measure<Voltage>> steerVoltages;
        public final List<Measure<Voltage>> wheelVoltages;
        public final List<Measure<Voltage>> angleSensorVoltages;
        public final List<Measure<Angle>> sensorAngles;
        public final Supplier<SwerveDriveWheelPositions> swerveDriveWheelPositions;
        public final Supplier<ChassisSpeeds> robotSpeeds;
        public final Consumer<SwerveDriveWheelStates> controlModules;
        public final Function<Translation2d, Consumer<ChassisSpeeds>> controlRobotChassisSpeeds;
        public final Runnable resetSteerEncodersFromAbsolutes;
        public final Runnable stop;
        public final Runnable update;

        private DriveSubsystem(
                        List<Measure<Voltage>> steerVoltages,
                        List<Measure<Voltage>> wheelVoltages,
                        List<Measure<Voltage>> angleSensorVoltages,
                        List<Measure<Angle>> sensorAngles,
                        Supplier<SwerveDriveWheelPositions> swerveDriveWheelPositions,
                        Supplier<ChassisSpeeds> robotSpeeds,
                        Consumer<SwerveDriveWheelStates> controlModules,
                        Function<Translation2d, Consumer<ChassisSpeeds>> controlRobotChassisSpeeds,
                        Runnable resetSteerEncodersFromAbsolutes,
                        Runnable stop,
                        Runnable update) {
                this.steerVoltages = steerVoltages;
                this.wheelVoltages = wheelVoltages;
                this.angleSensorVoltages = angleSensorVoltages;
                this.sensorAngles = sensorAngles;
                this.swerveDriveWheelPositions = swerveDriveWheelPositions;
                this.robotSpeeds = robotSpeeds;
                this.controlModules = controlModules;
                this.controlRobotChassisSpeeds = controlRobotChassisSpeeds;
                this.resetSteerEncodersFromAbsolutes = resetSteerEncodersFromAbsolutes;
                this.stop = stop;
                this.update = update;

                Command defaultCommand = run(stop);
                defaultCommand.setName("STOP");
                this.setDefaultCommand(defaultCommand);
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
                                () -> GoatMath.round(Constants.maxLinearVelocity.in(MetersPerSecond), 3),
                                null);

                builder.addDoubleProperty(
                                "Max Angular Velocity (mps)",
                                () -> GoatMath.round(Constants.maxAngularVelocity.in(RadiansPerSecond), 3),
                                null);
                builder.addDoubleProperty(
                                "Robot Forward Velocity (mps)",
                                () -> GoatMath.round(robotSpeeds.get().vxMetersPerSecond, 3),
                                null);

                builder.addDoubleProperty(
                                "Robot Strafe Velocity (mps)",
                                () -> GoatMath.round(robotSpeeds.get().vyMetersPerSecond, 3),
                                null);

                builder.addDoubleProperty(
                                "Robot Rotational Velocity (dps)",
                                () -> GoatMath.round(Math.toDegrees(robotSpeeds.get().omegaRadiansPerSecond), 3),
                                null);
                builder.addDoubleProperty(
                                String.format("Steer Voltage [%s]", 0),
                                () -> GoatMath.round(steerVoltages.get(0).in(Volts), 3),
                                null);
                builder.addDoubleProperty(
                                String.format("Steer Voltage [%s]", 1),
                                () -> GoatMath.round(steerVoltages.get(1).in(Volts), 3),
                                null);
                builder.addDoubleProperty(
                                String.format("Steer Voltage [%s]", 2),
                                () -> GoatMath.round(steerVoltages.get(2).in(Volts), 3),
                                null);
                builder.addDoubleProperty(
                                String.format("Steer Voltage [%s]", 3),
                                () -> GoatMath.round(steerVoltages.get(3).in(Volts), 3),
                                null);
                builder.addDoubleProperty(
                                String.format("Wheel Voltage [%s]", 0),
                                () -> GoatMath.round(wheelVoltages.get(0).in(Volts), 3),
                                null);
                builder.addDoubleProperty(
                                String.format("Wheel Voltage [%s]", 1),
                                () -> GoatMath.round(wheelVoltages.get(1).in(Volts), 3),
                                null);
                builder.addDoubleProperty(
                                String.format("Wheel Voltage [%s]", 2),
                                () -> GoatMath.round(wheelVoltages.get(2).in(Volts), 3),
                                null);
                builder.addDoubleProperty(
                                String.format("Wheel Voltage [%s]", 3),
                                () -> GoatMath.round(wheelVoltages.get(3).in(Volts), 3),
                                null);
                builder.addDoubleProperty(
                                String.format("Angle Sensor Voltage [%s]", 0),
                                () -> GoatMath.round(angleSensorVoltages.get(0).in(Volts), 3),
                                null);
                builder.addDoubleProperty(
                                String.format("Angle Sensor Voltage [%s]", 1),
                                () -> GoatMath.round(angleSensorVoltages.get(1).in(Volts), 3),
                                null);
                builder.addDoubleProperty(
                                String.format("Angle Sensor Voltage [%s]", 2),
                                () -> GoatMath.round(angleSensorVoltages.get(2).in(Volts), 3),
                                null);
                builder.addDoubleProperty(
                                String.format("Angle Sensor Voltage [%s]", 3),
                                () -> GoatMath.round(angleSensorVoltages.get(3).in(Volts), 3),
                                null);
                builder.addDoubleProperty(
                                String.format("Angle Sensor Voltage [%s]", 0),
                                () -> GoatMath.round(angleSensorVoltages.get(0).in(Volts), 3),
                                null);
                builder.addDoubleProperty(
                                String.format("Angle Sensor Voltage [%s]", 1),
                                () -> GoatMath.round(angleSensorVoltages.get(1).in(Volts), 3),
                                null);
                builder.addDoubleProperty(
                                String.format("Angle Sensor Voltage [%s]", 2),
                                () -> GoatMath.round(angleSensorVoltages.get(2).in(Volts), 3),
                                null);
                builder.addDoubleProperty(
                                String.format("Angle Sensor Voltage [%s]", 3),
                                () -> GoatMath.round(angleSensorVoltages.get(3).in(Volts), 3),
                                null);

                builder.addDoubleProperty(
                                String.format("Angle Sensor [%s] Degrees", 0),
                                () -> GoatMath.round(sensorAngles.get(0).in(Degrees), 3),
                                null);
                builder.addDoubleProperty(
                                String.format("Angle Sensor [%s] Degrees", 1),
                                () -> GoatMath.round(sensorAngles.get(1).in(Degrees), 3),
                                null);
                builder.addDoubleProperty(
                                String.format("Angle Sensor [%s] Degrees", 2),
                                () -> GoatMath.round(sensorAngles.get(2).in(Degrees), 3),
                                null);
                builder.addDoubleProperty(
                                String.format("Angle Sensor [%s] Degrees", 3),
                                () -> GoatMath.round(sensorAngles.get(3).in(Degrees), 3),
                                null);
        }

        public static final Supplier<DriveSubsystem> create = () -> {

                List<SwerveModule> modules = Arrays.asList(0, 1, 2, 3).stream().map(
                                (i) -> {
                                        TalonFXConfiguration config = new TalonFXConfiguration();
                                        config.Voltage.PeakForwardVoltage = 12.0;
                                        config.Voltage.PeakReverseVoltage = -12.0;
                                        config.TorqueCurrent.PeakForwardTorqueCurrent = 40;
                                        config.TorqueCurrent.PeakReverseTorqueCurrent = -40;
                                        config.Slot0.kP = Constants.wheelkPs.get(i);
                                        config.Slot0.kI = Constants.wheelkIs.get(i);
                                        config.Slot0.kD = Constants.wheelkDs.get(i);
                                        config.Slot0.kV = Constants.wheelkFs.get(i);
                                        config.Slot1.kP = Constants.wheelFOCkPs.get(i);
                                        config.Slot1.kI = Constants.wheelFOCkIs.get(i);
                                        config.Slot1.kD = Constants.wheelFOCkDs.get(i);
                                        config.Slot1.kV = Constants.wheelFOCkFs.get(i);
                                        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
                                        TalonFX wheelTalonFX = Motor.CTRE.createTalonFX.apply("CANIVORE")
                                                        .apply(Constants.wheelDeviceIds.get(i));
                                        wheelTalonFX.getConfigurator().apply(config);
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
                                                                                        Motor.REV.createMotorFromCANSparkBase)
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
                                                        .apply(Motor.CTRE.createMotorFromTalonFX
                                                                        .andThen(Motor.CTRE.setKrakenX60FOCMaxVelocity)
                                                                        .apply(wheelTalonFX))
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
                                                chassisSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.1);
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

                Runnable update = () -> {
                        Arrays.asList(0, 1, 2, 3).stream().forEachOrdered((i) -> {
                                modules.get(i).update.run();
                        });
                };

                return new DriveSubsystem(
                                steerVoltages,
                                wheelVoltages,
                                angleSensorVoltages,
                                sensorAngles,
                                currentWheelPositions,
                                robotSpeeds,
                                controlModuleStates,
                                controlRobotChassisSpeeds,
                                resetSteerEncodersFromAbsolutes,
                                stop,
                                update);
        };
}
