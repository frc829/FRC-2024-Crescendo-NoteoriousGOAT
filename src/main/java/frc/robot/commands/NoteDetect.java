package frc.robot.commands;

import static frc.robot.RobotContainer.telemetrySubsystem;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;

public abstract class NoteDetect {

        public static final Supplier<Command> create = () -> {
                Command setObjectDetectModeCommand = Commands.runOnce(
                                RobotContainer.telemetrySubsystem.enableObjectDetectors.get(0)::run,
                                RobotContainer.telemetrySubsystem);

                Command waitUntilNoteDetected = Commands.waitUntil(() -> {
                        return telemetrySubsystem.objectPositions.get(0).getSecond().get().isPresent();
                });

                Command goToNoteCommand = Commands.deferredProxy(DriveCommands.goToNoteCommandSupplier);

                Command setFieldDetectModeCommand = Commands.runOnce(
                                RobotContainer.telemetrySubsystem.enableFieldDetectors.get(0)::run,
                                RobotContainer.telemetrySubsystem);

                Command pickupDetectedNote = Commands.parallel(goToNoteCommand, Ground.groundCommandThenLevel.get());

                Command command = Commands.sequence(setObjectDetectModeCommand, waitUntilNoteDetected,
                                pickupDetectedNote,
                                setFieldDetectModeCommand);

                command.setName("Note Detect");
                return command;
        };

        public static final Supplier<Command> createForAuto = () -> {
                Command setObjectDetectModeCommand = Commands.runOnce(
                                RobotContainer.telemetrySubsystem.enableObjectDetectors.get(0)::run,
                                RobotContainer.telemetrySubsystem);

                Command waitUntilNoteDetected = Commands.waitUntil(() -> {
                        return telemetrySubsystem.objectPositions.get(0).getSecond().get().isPresent();
                });

                Command goToNoteCommand = Commands.deferredProxy(DriveCommands.goToNoteCommandSupplier);

                Command setFieldDetectModeCommand = Commands.runOnce(
                                RobotContainer.telemetrySubsystem.enableFieldDetectors.get(0)::run,
                                RobotContainer.telemetrySubsystem);

                Command pickupDetectedNote = Commands.race(goToNoteCommand, Ground.groundCommand.get());

                Command command = Commands.sequence(
                                setObjectDetectModeCommand,
                                waitUntilNoteDetected,
                                pickupDetectedNote,
                                setFieldDetectModeCommand);

                command.setName("Note Detect");
                return command;
        };

}
