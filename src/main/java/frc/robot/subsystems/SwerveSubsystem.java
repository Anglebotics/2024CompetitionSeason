package frc.robot.subsystems;

import java.lang.reflect.Field;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.lib.util.FieldUtil;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class SwerveSubsystem extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier simNotifier = null;
    private double lastSimTime;

    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();

//    public SwerveSubsystem(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
//        super(driveTrainConstants, OdometryUpdateFrequency, modules);
//        configPathPlanner();
//
//        if (Utils.isSimulation()) {
//            startSimThread();
//        }
//    }

    public SwerveSubsystem(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        configPathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }

        for (SwerveModule module : this.Modules) {
            CurrentLimitsConfigs driveMotorLimits = new CurrentLimitsConfigs();
            TalonFX driveMotor = module.getDriveMotor();
            driveMotor.getConfigurator().refresh(driveMotorLimits);
            driveMotorLimits.SupplyCurrentLimitEnable = true;
            driveMotorLimits.SupplyCurrentLimit = 45;
            driveMotor.getConfigurator().apply(driveMotorLimits);

            CurrentLimitsConfigs steerMotorLimits = new CurrentLimitsConfigs();
            TalonFX steerMotor = module.getSteerMotor();
            steerMotor.getConfigurator().refresh(steerMotorLimits);
            steerMotorLimits.SupplyCurrentLimitEnable = true;
            steerMotorLimits.SupplyCurrentLimit = 15;
            steerMotor.getConfigurator().apply(steerMotorLimits);
        }
    }

    @Override
    public void periodic() {
        Pose2d pose = m_odometry.getEstimatedPosition();
        SmartDashboard.putNumberArray("pose estimation", new Double[]{pose.getX(), pose.getY(), pose.getRotation().getDegrees()});
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public void zeroGyro() {
        m_pigeon2.setYaw(0);
    }

    private void startSimThread() {
        lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - lastSimTime;
            lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        simNotifier.startPeriodic(kSimLoopPeriod);
    }

    public void configPathPlanner() {
        AutoBuilder.configureHolonomic(
                () -> {
                    Pose2d curPose = getState().Pose;
                    if (!FieldUtil.isAllianceBlue()) {
                        curPose = new Pose2d(curPose.getTranslation(), curPose.getRotation().plus(Rotation2d.fromDegrees(180)));
                    }

                    return curPose;
                }, // Robot pose supplier
                this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getCurrentRobotChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::setChassisSpeedsAuto, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                Constants.AutoConstants.autoConfig,
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );
    }

    public void resetPose(Pose2d pose) {
        this.seedFieldRelative(pose);
        this.zeroGyroAdjusted(pose.getRotation());
    }

    public void setGyroYaw(Rotation2d yaw) {
        m_pigeon2.setYaw(yaw.getDegrees());
    }

    /**
     * Will rotate the provided value by 180 if on red alliance
     */
    public void zeroGyroAdjusted(Rotation2d rot) {
        setGyroYaw(FieldUtil.isAllianceBlue() ? rot : rot.plus(Rotation2d.fromDegrees(180)));
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    /**
     * Hijacks the robot's rotation but keeps PathPlanner translation. Used for
     * shooting while following the path
     *
     * @param speeds
     */
    public void setChassisSpeedsAuto(ChassisSpeeds speeds) {
        ChassisSpeeds newChassisSpeeds;
        if (FieldUtil.isAllianceBlue()) {
            newChassisSpeeds = new ChassisSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
        } else {
            newChassisSpeeds = new ChassisSpeeds(-speeds.vxMetersPerSecond, -speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
        }
        setControl(autoRequest.withSpeeds(newChassisSpeeds));
    }
}
