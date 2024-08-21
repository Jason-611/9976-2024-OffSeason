package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pitch;

public class PitchDown extends Command {
    private final Pitch pitch;

    public PitchDown(Pitch pitch) {
        this.pitch = pitch;
        addRequirements(pitch);
    }

    @Override
    public void execute() {
        double currentRad = pitch.getArmAngleRad();
        if (pitch.lock && currentRad > Pitch.LOWEST_POSITION) {
            double goalRadians = currentRad - Pitch.MOVE_ANGDEG;
            pitch.runSetPointProfiled(goalRadians);
        }
    }
}