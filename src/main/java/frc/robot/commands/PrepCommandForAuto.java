
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

/**
 * Prepare the shooter for a specific angle and speed
 */
public class PrepCommandForAuto extends Command {
    private final Shooter shooter;
    private double angle;
    private double speed;

    public PrepCommandForAuto(Shooter shooter, double angle, double speed) {
        addRequirements(shooter);
        this.shooter = shooter;
        this.angle = angle;
        this.speed = speed;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        shooter.setAngle(angle);
        shooter.startMotors(speed);
        shooter.setNoteReadyToFire(true);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (Utils.isSimulation()) {
            return true;
        }
        return shooter.canShoot();
    }
}
