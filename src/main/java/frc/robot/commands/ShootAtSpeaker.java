package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShootAtSpeaker extends Command {
    private final Shooter shooter;

    public ShootAtSpeaker(Shooter shooter) {
        this.shooter = shooter;

        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.runSpeakerShot();
    }
}