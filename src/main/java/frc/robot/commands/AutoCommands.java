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
        NamedCommands.registerCommand("FenderShoot", ScoringCommands.createFender.get());
        NamedCommands.registerCommand("Pickup", PickupCommands.createGround.get());
        NamedCommands.registerCommand("Amp", ScoringCommands.createAmpPosition.get());
        NamedCommands.registerCommand("GetLow",
                ResetAndHoldingCommands.setElevatorTiltForever.apply(Meters.of(0.0)).apply(Degrees.of(0)));
        NamedCommands.registerCommand("2NoteTopStart",
                TelemetryCommands.createSetStartPoseCommand.apply(TelemetryCommands.Constants.TwoNoteTopStart));
        NamedCommands.registerCommand("2NoteMidStart",
                TelemetryCommands.createSetStartPoseCommand.apply(TelemetryCommands.Constants.TwoNoteMidStart));
        NamedCommands.registerCommand("2NoteBottomStart",
                TelemetryCommands.createSetStartPoseCommand.apply(TelemetryCommands.Constants.TwoNoteMidBottom));
        NamedCommands.registerCommand("AmpStart",
                TelemetryCommands.createSetStartPoseCommand.apply(TelemetryCommands.Constants.AmpStart));
    }

}
