// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class PrepCommand extends Command {

    private final Shooter shooter;
    private double angle = 0;
    private double speed = 0;

    /**
     * Creates a new PrepCommand.
     */
    public PrepCommand(Shooter shooter, double angle, double speed) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(shooter);
        this.shooter = shooter;
        this.angle = angle;
        this.speed = speed;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        shooter.setAngle(angle);
        shooter.startMotors(speed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}
