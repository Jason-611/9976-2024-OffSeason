package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hang;

public class LooseHook extends Command {
    private final Hang hang;

    public LooseHook(Hang hang) {
        this.hang = hang;
        addRequirements(hang);
    }

    @Override
    public void execute() {
        hang.runSetPointProfiled(Math.toRadians(Hang.LOOSE_POSITION));
    }
}