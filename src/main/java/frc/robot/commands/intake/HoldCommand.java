package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

/**
 * This class does nothing and is expected to be interrupted by our shoot command.
 */
public class HoldCommand extends Command {

    public HoldCommand(Intake intake) {
        addRequirements(intake);
    }
}
