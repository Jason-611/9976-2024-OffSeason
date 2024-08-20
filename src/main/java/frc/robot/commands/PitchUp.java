package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pitch;

public class PitchUp extends Command {
    private final Pitch pitch;

    public PitchUp(Pitch pitch) {
        this.pitch = pitch;
        addRequirements(pitch);
    }

    @Override
    public void execute() {
        double goalRadians = pitch.getArmAngleRad() + Pitch.MOVE_ANGDEG;
        pitch.runSetPointProfiled(goalRadians);
    }
}