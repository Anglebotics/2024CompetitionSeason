package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.commands.intake.*;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class CommandHolder {
    private final Intake intake;
    private final Shooter shooter;
    private final GenericHID controller;

    private WaitForIntakeCommand waitForIntakeCommand = null;
    private IntakeNoteCommand intakeNoteCommand = null;
    private IntakeNoteCommand intakeNoteAndKeepRunningCommand = null;
    private MoveNoteToHoldingCommand moveNoteToHoldingCommand = null;
    private MoveNoteBackToShooterReadyCommand moveNoteBackToShooterReadyCommand = null;
    private HoldCommand holdCommand = null;

    public CommandHolder(
            Intake intake,
            Shooter shooter,
            GenericHID controller
    ) {
        this.intake = intake;
        this.shooter = shooter;
        this.controller = controller;
    }

    public WaitForIntakeCommand waitForIntakeCommand() {
        if (waitForIntakeCommand == null) {
            waitForIntakeCommand = new WaitForIntakeCommand(this, intake);
        }
        return waitForIntakeCommand;
    }

    public IntakeNoteCommand intakeNoteCommand() {
        if (intakeNoteCommand == null) {
            intakeNoteCommand = new IntakeNoteCommand(this, intake, false);
        }
        return intakeNoteCommand;
    }

    public IntakeNoteCommand intakeNoteAndKeepRunningCommand() {
        if (intakeNoteAndKeepRunningCommand == null) {
            intakeNoteAndKeepRunningCommand = new IntakeNoteCommand(this, intake, true);
        }
        return intakeNoteAndKeepRunningCommand;
    }

    public MoveNoteToHoldingCommand moveNoteToHoldingCommand() {
        if (moveNoteToHoldingCommand == null) {
            moveNoteToHoldingCommand = new MoveNoteToHoldingCommand(this, intake, shooter);
        }
        return moveNoteToHoldingCommand;
    }

    public MoveNoteBackToShooterReadyCommand moveNoteBackToShooterReadyCommand() {
        if (moveNoteBackToShooterReadyCommand == null) {
            moveNoteBackToShooterReadyCommand = new MoveNoteBackToShooterReadyCommand(this, intake, shooter);
        }
        return moveNoteBackToShooterReadyCommand;
    }

    public HoldCommand holdCommand() {
        if (holdCommand == null) {
            holdCommand = new HoldCommand(intake);
        }
        return holdCommand;
    }

    public RumbleCommand rumbleCommand(RumbleCommand.Power power, RumbleCommand.Time time) {
        return new RumbleCommand(controller, power, time);
    }
}
