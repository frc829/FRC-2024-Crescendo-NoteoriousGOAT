package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics.SwerveDriveWheelStates;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.commandCreators.BasicCommands;
import frc.robot.commandCreators.DriveCommands;
import frc.robot.commandCreators.PickupCommands;
import frc.robot.commandCreators.ResetAndHoldingCommands;
import frc.robot.commandCreators.ScoringCommands;

public class ManualCommands {
        public static final class Elevator {
                public static final Command drive = BasicCommands.Set.ElevatorDrive.create
                                .apply(RobotContainer.operator.leftYValue);
        }

        public static final class Tilt {
                public static final Command drive = BasicCommands.Set.TiltDrive.create
                                .apply(() -> RobotContainer.operator.rightYValue.getAsDouble() * 0.5);
        }

        public static final class Shooter {
                public static final Runnable shooterSpin = () -> {
                        RobotContainer.topShooterSubsystem.spin
                                        .accept(RobotContainer.operator.fullTriggerValue.getAsDouble());
                        RobotContainer.bottomShooterSubsystem.spin
                                        .accept(RobotContainer.operator.fullTriggerValue.getAsDouble());
                };
                public static final Command command = Commands.run(shooterSpin,
                                RobotContainer.topShooterSubsystem,
                                RobotContainer.bottomShooterSubsystem);
        }

        public static final class Drive {
                public static final class ZeroModules {
                        private static final SwerveDriveWheelStates wheelStates = new SwerveDriveWheelStates(
                                        new SwerveModuleState[] {
                                                        new SwerveModuleState(),
                                                        new SwerveModuleState(),
                                                        new SwerveModuleState(),
                                                        new SwerveModuleState()
                                        });
                        private static final Runnable zero = () -> RobotContainer.driveSubsystem.controlModules
                                        .accept(wheelStates);
                        public static final Command command = Commands.run(zero, RobotContainer.driveSubsystem);
                }

                public static final class FieldCentric {
                        public static final Command command = DriveCommands.createFieldCentricCommand.get();
                        public static final Command frontLeft = DriveCommands.createFieldCentricFLCommand.get();
                        public static final Command frontRight = DriveCommands.createFieldCentricFRCommand.get();

                }

                public static final class RobotCentric {
                        public static final Command command = DriveCommands.createRobotCentricCommand.get();
                        public static final Command frontLeftRC = DriveCommands.createRobotCentricFLCommand.get();
                        public static final Command frontRightRC = DriveCommands.createRobotCentricFRCommand.get();
                }

                public static final class Positions {
                        public static final Command source = DriveCommands.createGoToSource.get();
                }
        }

        public static final class Pickup {
                public static final Command barf = PickupCommands.createBarf.get();
                public static final Command groundPickup = PickupCommands.createGround.get();
                public static final Command groundPickupReset = ResetAndHoldingCommands.setElevatorTiltUntil
                                .apply(Meters.of(0))
                                .apply(Degrees.of(0));
                public static final Command babyBirdPickup = PickupCommands.createGround.get();
                public static final Command babyBirdPickupReset = ResetAndHoldingCommands.setElevatorTiltUntil
                                .apply(Meters.of(0))
                                .apply(Degrees.of(0));
                public static final Command noteDetectPickup = PickupCommands.createNoteDetect.get();
                public static final Command noteDetectReset = ResetAndHoldingCommands.setElevatorTiltUntil
                                .apply(Meters.of(0))
                                .apply(Degrees.of(0));
        }

        public static final class Scoring {
                public static final Command ampScore = ScoringCommands.createAmp.get();
                public static final Command ampReset = ResetAndHoldingCommands.setElevatorTiltUntil
                                .apply(Meters.of(0))
                                .apply(Degrees.of(0));
                public static final Command fenderScore = ScoringCommands.createFender.get();
                public static final Command fenderReset = ResetAndHoldingCommands.setElevatorTiltUntil
                                .apply(Meters.of(0))
                                .apply(Degrees.of(0));
                public static final Command rangedScore = ScoringCommands.createRanged.get();
                public static final Command rangedReset = ResetAndHoldingCommands.setElevatorTiltForever
                                .apply(Meters.of(0))
                                .apply(Degrees.of(0));
        }

        public static final class ResetAndHolding {
                public static final Command shooterAdjust = ResetAndHoldingCommands.distanceBasedShooterAdjust.get();

        }

        static {
                Elevator.drive.setName("Manual Elevator");
                Tilt.drive.setName("Manual Tilt");
                Shooter.command.setName("Manual Shooter");
                Drive.FieldCentric.command.setName("Manual Field Centric");
                Drive.ZeroModules.command.setName("Zero Modules");
        }
}
