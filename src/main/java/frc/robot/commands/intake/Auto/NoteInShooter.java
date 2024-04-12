// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake.Auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class NoteInShooter extends Command {
    Intake intake;
    Shooter shooter;

    /** Creates a new StartIntake. */
    public NoteInShooter(Intake intake, Shooter shooter) {
        this.intake = intake;
        addRequirements(intake);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        shooter.readyArmForNewNote();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        boolean noteAtShooterSensor = intake.noteAtShooterSensor();
        return !noteAtShooterSensor;
    }
}
