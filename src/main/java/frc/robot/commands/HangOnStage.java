package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hang;

public class HangOnStage extends Command {
    private final Hang hang;

    public HangOnStage(Hang hang) {
        this.hang = hang;

        addRequirements(hang);
    }

    @Override
    public void execute() {
        hang.runSetPointProfiled(Math.toRadians(Hang.TIGHT_POSITION));
    }
}