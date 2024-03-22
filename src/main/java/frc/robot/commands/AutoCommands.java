package frc.robot.commands;

import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.commandCreators.PickupCommands;
import frc.robot.commandCreators.StationaryRangedShot;
import frc.robot.commandCreators.BasicScoringCommands;
import frc.robot.commandCreators.TelemetryCommands;

public class AutoCommands {

        public AutoCommands() {
                NamedCommands.registerCommand("BackReset",
                                TelemetryCommands.createResetPoseFromBackCameraCommand.get());
                NamedCommands.registerCommand("Fender", BasicScoringCommands.Fender.createWithDelay.get());
                NamedCommands.registerCommand("Ranged", StationaryRangedShot.Ranged.createWithDelay.get());
                NamedCommands.registerCommand("Pass", BasicScoringCommands.Pass.createWithDelay.get());

                NamedCommands.registerCommand("PBJ1", BasicScoringCommands.PBJ1.createWithDelay.get());
                NamedCommands.registerCommand("PBJ2", BasicScoringCommands.PBJ2.createWithDelay.get());
                NamedCommands.registerCommand("PBJ3", BasicScoringCommands.PBJ3.createWithDelay.get());
                NamedCommands.registerCommand("PBJ4", BasicScoringCommands.PBJ4.createWithDelay.get());
                NamedCommands.registerCommand("PBJ5", BasicScoringCommands.PBJ5.createWithDelay.get());
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
                NamedCommands.registerCommand("PBJStart",
                                TelemetryCommands.createSetStartPoseCommand
                                                .apply(TelemetryCommands.Constants.pbjStart));
                NamedCommands.registerCommand("RearToField",
                                TelemetryCommands.createSetRearCameraToFieldCommand.get());
        }

}
