// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake.Auto;

import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class StartIntake extends Command {
    private final VoltageOut voltageOut = new VoltageOut(0.0);
    Intake intake;
    Shooter shooter;

    /** Creates a new StartIntake. */
    public StartIntake(Intake intake, Shooter shooter) {
        this.intake = intake;
        addRequirements(intake);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        shooter.readyArmForNewNote();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        intake.setMotorControl(voltageOut.withOutput(4)); // Chosen by fair dice roll
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        boolean noteAtPreIntakeSensor = intake.noteAtPreIntakeSensor();
        boolean noteAtIntakeSensor = intake.noteAtIntakeSensor();
        boolean noteAtShooterSensor = intake.noteAtShooterSensor();
        boolean noteAtAnySensor = noteAtPreIntakeSensor || noteAtIntakeSensor || noteAtShooterSensor;
        return noteAtAnySensor;
    }
}
