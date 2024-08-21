package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Pitch;
import frc.robot.subsystems.Intake;

public class NoteStucked extends SequentialCommandGroup {
    public NoteStucked(Pitch pitch, Intake intake) {
        super.addRequirements(pitch, intake);

        addCommands(Commands.run(() -> pitch.runSetPointProfiled(Pitch.NOTE_STUCK_RAD), pitch)
            .beforeStarting(() -> pitch.runSetPointProfiled(Pitch.NOTE_STUCK_RAD))
            .until(pitch::inPosition));

        addCommands(Commands.run(intake::runReverse, intake)
            .alongWith(Commands.run(() -> pitch.runSetPointProfiled(Pitch.NOTE_STUCK_RAD), pitch))
        );
    }
}