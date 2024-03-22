package frc.robot.commands.intake;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class EjectCommand extends Command {
    private final Intake intake;

    private final VoltageOut voltageOut = new VoltageOut(0.0);
    private final NeutralOut brake = new NeutralOut();

    public EjectCommand(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        intake.setMotorControl(voltageOut.withOutput(-1.2));
    }

    @Override
    public void end(boolean interrupted) {
        intake.setMotorControl(brake);
    }

    @Override
    public boolean isFinished() {
        return !intake.noteSensedInIntake();
    }
}
