package frc.robot.commands.intake;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.CommandHolder;
import frc.robot.commands.RumbleCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class MoveNoteBackToShooterReadyCommand extends Command {
    private final CommandHolder commands;
    private final Intake intake;
    private final Shooter shooter;

    private final Timer timer = new Timer();

    private final VoltageOut voltageOut = new VoltageOut(0.0);
    private final NeutralOut brake = new NeutralOut();

    public MoveNoteBackToShooterReadyCommand(
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
        timer.restart();
        intake.setMotorControl(voltageOut.withOutput(-1.2));  // Move the note back slightly
    }

    @Override
    public void end(boolean interrupted) {
        intake.setMotorControl(brake);
        shooter.setNoteInShooter(true);
        commands.holdCommand().schedule();
        commands.rumbleCommand(RumbleCommand.Power.LOW, RumbleCommand.Time.FAST).schedule();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(0.1);
    }
}
