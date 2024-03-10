package frc.robot.commands;

import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.commandCreators.PickupCommands;
import frc.robot.commandCreators.BasicScoringCommands;
import frc.robot.commandCreators.TelemetryCommands;

public class AutoCommands {

        public AutoCommands() {
                NamedCommands.registerCommand("BackReset",
                                TelemetryCommands.createResetPoseFromBackCameraCommand.get());
                NamedCommands.registerCommand("Fender", BasicScoringCommands.Fender.createWithDelay.get());
                NamedCommands.registerCommand("AutoRanged1", BasicScoringCommands.AutoRanged1.createWithDelay.get());
                NamedCommands.registerCommand("AutoRanged2", BasicScoringCommands.AutoRanged2.createWithDelay.get());
                NamedCommands.registerCommand("AutoRanged3", BasicScoringCommands.AutoRanged3.createWithDelay.get());
                NamedCommands.registerCommand("Pickup", PickupCommands.Ground.groundCommand.get());
                NamedCommands.registerCommand("SpinUp", BasicScoringCommands.SpinUp.createSpinUp.get());
                NamedCommands.registerCommand("GetLow", PickupCommands.Level.command.get());
                NamedCommands.registerCommand("SpeakerTopStart",
                                TelemetryCommands.createSetStartPoseCommand
                                                .apply(TelemetryCommands.Constants.SpeakerTopStart));
                NamedCommands.registerCommand("SpeakerMidStart",
                                TelemetryCommands.createSetStartPoseCommand
                                                .apply(TelemetryCommands.Constants.SpeakerMidStart));
                NamedCommands.registerCommand("SpeakerBotStart",
                                TelemetryCommands.createSetStartPoseCommand
                                                .apply(TelemetryCommands.Constants.SpeakerBotStart));
                NamedCommands.registerCommand("AmpStart",
                                TelemetryCommands.createSetStartPoseCommand
                                                .apply(TelemetryCommands.Constants.AmpStart));
        }

}
