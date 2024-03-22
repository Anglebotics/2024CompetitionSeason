package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.CommandHolder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class MoveNoteToHoldingCommand extends Command {
    private final CommandHolder commands;
    private final Intake intake;
    private final Shooter shooter;

    public MoveNoteToHoldingCommand(
            CommandHolder commands,
            Intake intake,
            Shooter shooter
    ) {
        this.commands = commands;
        this.intake = intake;
        this.shooter = shooter;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        shooter.readyArmForNewNote();
    }

    @Override
    public void execute() {
        boolean noteAtShooterSensor = intake.noteAtShooterSensor();

        if(! noteAtShooterSensor) {  // We've moved the note past this sensor
            commands.moveNoteBackToShooterReadyCommand().schedule();
        }
    }
}
