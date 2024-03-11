// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.Constants;

import com.ctre.phoenix6.hardware.TalonFX;

public class Intake extends SubsystemBase {
    private final DigitalInput preIntakeSensor = new DigitalInput(7);
    private final DigitalInput intakeSensor = new DigitalInput(8);
    private final DigitalInput shooterSensor = new DigitalInput(9);

    private final TalonFX intakeMotor = new TalonFX(4, Constants.OperatorConstants.canivoreSerial);
    private final Servo RightIntakeServo = new Servo(1); // PWM channel is likely subject to change.
    private final Servo LeftIntakeServo = new Servo(2); // PWM channel is likely subject to change.

    private boolean notePassedShooterSensor = false;

    private boolean status = intakeMotor.isAlive();

    private enum IntakeState {
        IDLE, // No motors are moving, no note is inside the mechanism
        INTAKING, // Motors are moving, note is not yet where we need it to be
        HOLDING, // No motors are moving, note is where it needs to be and is contained in the robot
        SHOOTING, // Motors are moving, note is being moved into the gears
    }

    private Timer timer = new Timer();

    private static IntakeState currentState = IntakeState.IDLE;

    public void intakeInit() {
        intakeMotor.setInverted(true);
        timer.stop();
        timer.reset();
    }

    public void optomizeCan() {
        TalonFX.optimizeBusUtilizationForAll(intakeMotor);
    }

    public void shoot(double speed) {
        LeftIntakeServo.setAngle(0); //Angle is subject to cahnge depending on the orientation of the servos.
        RightIntakeServo.setAngle(0); //Angle is subject to cahnge depending on the orientation of the servos.
        currentState = IntakeState.SHOOTING;        
        intakeMotor.set(speed);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Motor Functional?", status);
        SmartDashboard.putNumber("Intake Motor Speed", intakeMotor.get());
        SmartDashboard.putBoolean("Pre Intake Sensor", preIntakeSensor.get());
        SmartDashboard.putBoolean("Intake Sensor", intakeSensor.get());
        SmartDashboard.putBoolean("Shooter Sensor", shooterSensor.get());
        SmartDashboard.putString("Intake State", currentState.name());
        SmartDashboard.putNumber("Timer", timer.get());
        boolean noteAtPreIntakeSensor = ! preIntakeSensor.get();
        boolean noteAtIntakeSensor = ! intakeSensor.get();
        boolean noteAtShooterSensor = ! shooterSensor.get();
        switch (currentState) {
            case IDLE:

                if (intakeMotor.get() != 0) { // If the intake motor is moving, stop it
                    intakeMotor.set(0);
                }

                if (noteAtPreIntakeSensor) { // If there is a note at the intake, start intaking and make sure that the timers are reset and stopped
                    LeftIntakeServo.setAngle(160);
                    RightIntakeServo.setAngle(0);

                    currentState = IntakeState.INTAKING;
                    notePassedShooterSensor = false;
                    timer.stop();
                    timer.reset();
                }

                break;
            case INTAKING:

                if (noteAtPreIntakeSensor || noteAtIntakeSensor || noteAtShooterSensor) { // If there is a note in the intake subsystem, make sure the motor is moving
                    if (intakeMotor.get() == 0) { // If the motor is not moving, make it move
                        intakeMotor.set(0.2);
                    }
                } else { // The note is not touching any of the sensors
                    if (! notePassedShooterSensor) { // If the note has gone not past the last sensor
                        currentState = IntakeState.IDLE;
                        intakeMotor.stopMotor();
                    }
                }

                if (noteAtShooterSensor) { // Now we know when the note hits the shooter sensor
                    if (! notePassedShooterSensor) {
                        notePassedShooterSensor = true;
                        timer.start();
                    }
                }

                if (timer.hasElapsed(0.1)) { // TODO: tune this value. This timer starts when the note hits the last sensor, and this value stops the intaking process when the timer reaches this value
                    currentState = IntakeState.IDLE;
                    intakeMotor.stopMotor();
                    timer.stop();
                    timer.reset();
                }

                break;
            case HOLDING:

                LeftIntakeServo.setAngle(160);
                RightIntakeServo.setAngle(0);
                intakeMotor.stopMotor();

                break;
            case SHOOTING:

                if (timer.get() == 0) { // If the timer has not started, start the timer and the motor
                    intakeMotor.set(0.2);                    
                    timer.start();
                }

                if (timer.hasElapsed(1)) { // If the intake has been running for 1 second, the note has shot. Reset everything back to idle
                    currentState = IntakeState.IDLE;
                    intakeMotor.stopMotor();
                    timer.stop();
                    timer.reset();
                }

                break;
        }
    }

    public void disable() {
        intakeMotor.stopMotor();
        timer.stop();
        timer.reset();
    }
}