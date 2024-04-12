// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake.Auto;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class MoveBack extends Command {
    private final Intake intake;
    private final Shooter shooter;

    private final Timer timer = new Timer();

    private final VoltageOut voltageOut = new VoltageOut(0.0);
    private final NeutralOut brake = new NeutralOut();

    public MoveBack(Intake intake, Shooter shooter) {
        this.intake = intake;
        this.shooter = shooter;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        timer.restart();
        intake.setMotorControl(voltageOut.withOutput(-1.2)); // Move the note back slightly
    }

    @Override
    public void end(boolean interrupted) {
        intake.setMotorControl(brake);
        shooter.setNoteInShooter(true);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(0.15);
    }
}
