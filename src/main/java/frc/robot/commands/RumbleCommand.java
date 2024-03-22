// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;

public class RumbleCommand extends Command {
    public enum Power {
        LOW(0.25),
        MEDIUM(0.5),
        HIGH(1.0);

        public final double powerValue;

        Power(double powerValue) {
            this.powerValue = powerValue;
        }
    }

    public enum Time {
        FAST(0.25);

        private final double timeInSeconds;

        Time(double timeInSeconds) {
            this.timeInSeconds = timeInSeconds;
        }
    }

    private final GenericHID controller;
    private final Power power;
    private final Time time;
    private final Timer timer;

    public RumbleCommand(GenericHID controller, Power power, Time time) {
        this.controller = controller;
        this.time = time;
        this.power = power;
        this.timer = new Timer();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if(DriverStation.isAutonomous()) {
            return;
        }
        controller.setRumble(RumbleType.kBothRumble, power.powerValue);
        timer.restart();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        controller.setRumble(RumbleType.kBothRumble, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return timer.hasElapsed(time.timeInSeconds) || DriverStation.isAutonomous();
    }
}
