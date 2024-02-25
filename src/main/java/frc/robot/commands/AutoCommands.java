package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.commandCreators.PickupCommands;
import frc.robot.commandCreators.ResetAndHoldingCommands;
import frc.robot.commandCreators.ScoringCommands;
import frc.robot.commandCreators.TelemetryCommands;

public class AutoCommands {

    public AutoCommands() {
        NamedCommands.registerCommand("RangedShoot", ScoringCommands.createRanged.get());
        NamedCommands.registerCommand("Fender", ScoringCommands.createFenderWithDelay.get());
        NamedCommands.registerCommand("Pickup", PickupCommands.createGroundNoLevel.get());
        NamedCommands.registerCommand("Amp", ScoringCommands.createAmpPosition.get());
        NamedCommands.registerCommand("SpinUp", ScoringCommands.createSpinUp.get());
        NamedCommands.registerCommand("GetLow",
                ResetAndHoldingCommands.setElevatorTiltForever.apply(Meters.of(0.0)).apply(Degrees.of(0)));
        NamedCommands.registerCommand("SpeakerTopStart",
                TelemetryCommands.createSetStartPoseCommand.apply(TelemetryCommands.Constants.SpeakerTopStart));
        NamedCommands.registerCommand("SpeakerMidStart",
                TelemetryCommands.createSetStartPoseCommand.apply(TelemetryCommands.Constants.SpeakerMidStart));
        NamedCommands.registerCommand("SpeakerBotStart",
                TelemetryCommands.createSetStartPoseCommand.apply(TelemetryCommands.Constants.SpeakerBotStart));
        NamedCommands.registerCommand("AmpStart",
                TelemetryCommands.createSetStartPoseCommand.apply(TelemetryCommands.Constants.AmpStart));
    }

}
