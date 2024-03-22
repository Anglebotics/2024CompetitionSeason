// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.generated.ArmTunerConstants;

public class Shooter extends SubsystemBase {
    private final TalonFX pivotMotor = new TalonFX(14, Constants.OperatorConstants.canivoreSerial);
    private final TalonFX leftShootMotor = new TalonFX(5, Constants.OperatorConstants.canivoreSerial);
    private final TalonFX rightShootMotor = new TalonFX(13, Constants.OperatorConstants.canivoreSerial);
    private final CANcoder armPositionEncoder = new CANcoder(18, Constants.OperatorConstants.canivoreSerial);

    public static final double PIVOT_MOTOR_ANGLE_ERROR_THRESHOLD_ID = 1.0 / 360.0;
    public static final double SHOOTER_MOTOR_VELOCITY_ERROR_THRESHOLD_ID = 4;

    private double targetArmPosition = 30;
    private double targetVelocity = 0;

    private final VelocityVoltage shooterVelocityControl = new VelocityVoltage(0).withEnableFOC(true);
    private final NeutralOut brake = new NeutralOut();
    private boolean noteIsInShooter = false;
    private boolean readyToFire = false;


    public CANcoder getPivotMotorEncoder() {
        return armPositionEncoder;
    }

    public void shooterInit() {
        leftShootMotor.setNeutralMode(NeutralModeValue.Brake);
        rightShootMotor.setNeutralMode(NeutralModeValue.Brake);

        leftShootMotor.setInverted(true);

        MagnetSensorConfigs wristPositionMagnetConfigs = new MagnetSensorConfigs();
        wristPositionMagnetConfigs.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        wristPositionMagnetConfigs.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        wristPositionMagnetConfigs.MagnetOffset = 0.73933;  // Calibrate this with CalibrateWristAngleCommand
        armPositionEncoder.getConfigurator().apply(wristPositionMagnetConfigs);

        FeedbackConfigs pivotConfigs = new FeedbackConfigs();
        pivotConfigs.FeedbackRemoteSensorID = armPositionEncoder.getDeviceID();
        pivotConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        pivotConfigs.RotorToSensorRatio = 45.0 / 8.0;
        pivotMotor.getConfigurator().apply(pivotConfigs);

        pivotMotor.getConfigurator().apply(ArmTunerConstants.pivotMotionMagicConfigs);
        pivotMotor.getConfigurator().apply(ArmTunerConstants.pivotPIDConfigs);

        SoftwareLimitSwitchConfigs pivotLimits = new SoftwareLimitSwitchConfigs();
        pivotLimits.ForwardSoftLimitThreshold = 90.0 / 360.0;
        pivotLimits.ReverseSoftLimitThreshold = 30.0 / 360.0;
        pivotLimits.ForwardSoftLimitEnable = true;
        pivotLimits.ReverseSoftLimitEnable = true;
        pivotMotor.getConfigurator().apply(pivotLimits);

        MotorOutputConfigs outputConfigs = new MotorOutputConfigs();
        outputConfigs.Inverted = InvertedValue.Clockwise_Positive;
        pivotMotor.getConfigurator().apply(outputConfigs);
        leftShootMotor.getConfigurator().apply(ArmTunerConstants.shooterPIDConfigs);
        rightShootMotor.getConfigurator().apply(ArmTunerConstants.shooterPIDConfigs);

        armPositionEncoder.getAbsolutePosition().setUpdateFrequency(200);
        leftShootMotor.getVelocity().setUpdateFrequency(50);
        rightShootMotor.getVelocity().setUpdateFrequency(50);
//    TalonFX.optimizeBusUtilizationForAll(pivotMotor, leftShootMotor, rightShootMotor);
        stopMotors();
        readyArmForNewNote();
    }


    public void setAngle(double angle) {
        pivotMotor.setControl(new PositionVoltage(angle / 360.0));
//    pivotMotor.setControl(motionMagicVoltage.withPosition(angle / 360));
        targetArmPosition = angle;
    }

    public void startMotors(double rotationsPerSecond) {
        VelocityVoltage vspeed = shooterVelocityControl.withVelocity(rotationsPerSecond);
        targetVelocity = rotationsPerSecond;
        rightShootMotor.setControl(vspeed);
        leftShootMotor.setControl(vspeed);

    }

    public boolean canShoot() {
        boolean armInCorrectPosition = Math.abs(pivotMotor.getClosedLoopError().getValue()) <= PIVOT_MOTOR_ANGLE_ERROR_THRESHOLD_ID;
        boolean leftMotorAtSpeed = Math.abs(leftShootMotor.getClosedLoopError().getValue()) <= SHOOTER_MOTOR_VELOCITY_ERROR_THRESHOLD_ID;
        boolean rightMotorAtSpeed = Math.abs(rightShootMotor.getClosedLoopError().getValue()) <= SHOOTER_MOTOR_VELOCITY_ERROR_THRESHOLD_ID;
        boolean motorsAtShootingSpeed = leftMotorAtSpeed && rightMotorAtSpeed;
        return armInCorrectPosition && motorsAtShootingSpeed;
//        return true;
    }

    public void readyArmForNewNote() {
        setAngle(30);
    }

    public void stopMotors() {
        rightShootMotor.setControl(brake);
        leftShootMotor.setControl(brake);
    }

    @Override
    public void periodic() {
        double motorVelocity = leftShootMotor.getVelocity().getValue();
        double positionOfArm = armPositionEncoder.getAbsolutePosition().getValue() * 360;
        double pivotError = pivotMotor.getClosedLoopError().getValue() * 360;

        String SMART_DASHBOARD_VELOCITY = "Shooter Motor Velocity";
        SmartDashboard.putNumber(SMART_DASHBOARD_VELOCITY, motorVelocity);
        String SMART_DASHBOARD_TARGET_VELOCITY = "Shooter Motor Target Velocity";
        SmartDashboard.putNumber(SMART_DASHBOARD_TARGET_VELOCITY, targetVelocity);
        String SMART_DASHBOARD_POSITION = "Shooter Motor Position";
        SmartDashboard.putNumber(SMART_DASHBOARD_POSITION, positionOfArm);
        String SMART_DASHBOARD_TARGET_POSITION = "Shooter Motor Target Position";
        SmartDashboard.putNumber(SMART_DASHBOARD_TARGET_POSITION, targetArmPosition);
        SmartDashboard.putNumber("Arm position rotations", armPositionEncoder.getPosition().getValue());
        SmartDashboard.putNumber("Wrist Error", pivotError);
    }

    public void setNoteInShooter(boolean noteInShooter) {
        noteIsInShooter = noteInShooter;
    }

    public boolean isNoteInShooter() {
        return noteIsInShooter;
    }

    public void setNoteReadyToFire(boolean readyToFire) {
        this.readyToFire = readyToFire;
    }

    public boolean isNoteReadyToFire() {
        return readyToFire;
    }
}
