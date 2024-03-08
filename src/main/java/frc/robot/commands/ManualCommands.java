package frc.robot.commands;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics.SwerveDriveWheelStates;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.commandCreators.DriveCommands;

public class ManualCommands {


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
}
