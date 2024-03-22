// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class ShootCommand extends Command {
    private final Shooter shooter;
    private final Intake intake;
    private final Timer timer = new Timer();

    private final VoltageOut voltageOut = new VoltageOut(0.0);
    private final NeutralOut brake = new NeutralOut();

    public ShootCommand(
            Shooter shooter,
            Intake intake
    ) {
        this.shooter = shooter;
        this.intake = intake;
        addRequirements(shooter);
        addRequirements(intake);
    }

    public void initialize() {
        intake.setMotorControl(voltageOut.withOutput(2.4));

        timer.restart();
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setNoteInShooter(false);
        shooter.setNoteReadyToFire(false);
        shooter.stopMotors();
        shooter.readyArmForNewNote();
        intake.setMotorControl(brake);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(1);
    }
}
