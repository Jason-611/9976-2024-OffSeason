package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pitch;

public class TogglePitchLock extends Command {
    private final Pitch pitch;

    public TogglePitchLock(Pitch pitch) {
        this.pitch = pitch;
        addRequirements(pitch);
    }

    @Override
    public void execute() {
        pitch.lock = !pitch.lock;
    }
}