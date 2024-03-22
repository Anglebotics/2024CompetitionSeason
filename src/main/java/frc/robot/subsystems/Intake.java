// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.ControlRequest;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

import com.ctre.phoenix6.hardware.TalonFX;

public class Intake extends SubsystemBase {
    private final DigitalInput preIntakeSensor = new DigitalInput(7);
    private final DigitalInput intakeSensor = new DigitalInput(8);
    private final DigitalInput shooterSensor = new DigitalInput(9);

    private final TalonFX intakeMotor = new TalonFX(4, Constants.OperatorConstants.canivoreSerial);

    public void intakeInit() {
        intakeMotor.setInverted(true);
    }

    public StatusCode setMotorControl(ControlRequest request) {
        return intakeMotor.setControl(request);
    }

    public boolean noteAtPreIntakeSensor() {
        return ! preIntakeSensor.get();
    }

    public boolean noteAtIntakeSensor() {
        return ! intakeSensor.get();
    }

    public boolean noteAtShooterSensor() {
        return ! shooterSensor.get();
    }

    /**
     *
     * @return Is the note tripping any of our sensors. Might return false if the note is in the shooter.
     */
    public boolean noteSensedInIntake() {
        return noteAtPreIntakeSensor() || noteAtIntakeSensor() || noteAtShooterSensor();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Motor Speed", intakeMotor.get());
        SmartDashboard.putBoolean("Pre Intake Sensor", preIntakeSensor.get());
        SmartDashboard.putBoolean("Intake Sensor", intakeSensor.get());
        SmartDashboard.putBoolean("Shooter Sensor", shooterSensor.get());
    }
}
