// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.*;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Eject;
import frc.robot.commands.PrepCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveSubsystem;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.auto.AutoBuilder;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Shooter shooterSubsystem = new Shooter();
  private final Intake intakeSubsystem = new Intake();

  private SendableChooser<Command> autoChooser = new SendableChooser<Command>();

  private final ShootCommand shootCommand = new ShootCommand(shooterSubsystem, intakeSubsystem);
  private final PrepCommand firstShoot = new PrepCommand(shooterSubsystem, 0, 0); //TODO Tune for actual angles
  private final PrepCommand secondShoot = new PrepCommand(shooterSubsystem, 0, 0); //TODO Tune for actual angles
  private final PrepCommand thirdShoot = new PrepCommand(shooterSubsystem, 0, 0); //TODO Tune for actual angles
  private final PrepCommand closeShoot = new PrepCommand(shooterSubsystem, 0, 0); //TODO Tune for actual angles
  private final PrepCommand ampShoot = new PrepCommand(shooterSubsystem, 0, 0); //TODO Tune for actual angles
  private final PrepCommand speakerShoot = new PrepCommand(shooterSubsystem, 0, 0); //TODO Tune for actual angles
  private final Eject eject = new Eject(intakeSubsystem, -0.2);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  public final CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */

  private double maxSpeed = 6; // 6 meters per second desired top speed
  private double maxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveSubsystem drivetrain = SwerveTunerConstants.DriveTrain; // My drivetrain

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(maxSpeed * 0.1).withRotationalDeadband(maxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry telemetry = new Telemetry(maxSpeed);
  
  public RobotContainer() {
    configureBindings();
    intakeSubsystem.intakeInit();
    shooterSubsystem.shooterInit();
    intakeSubsystem.optomizeCan();
    drivetrain.optimizeCan();
    
    NamedCommands.registerCommand("First Shoot", firstShoot);
    NamedCommands.registerCommand("Second Shoot", secondShoot);
    NamedCommands.registerCommand("Third Shoot", thirdShoot);
    NamedCommands.registerCommand("Close First", closeShoot);
    
    autoChooser = AutoBuilder.buildAutoChooser("Center 2pt");
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    driverController.a().onTrue(ampShoot);
    driverController.b().onTrue(speakerShoot);
    driverController.leftTrigger().onTrue(eject);
    driverController.rightTrigger().onTrue(shootCommand);

    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-driverController.getLeftY() * maxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-driverController.getLeftX() * maxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-driverController.getRightX() * maxAngularRate) // Drive counterclockwise with negative X (left)
        ));

    driverController.back().whileTrue(drivetrain.applyRequest(() -> brake));
    driverController.start().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))));

    // reset the field-centric heading on left bumper press
    driverController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));


    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    
    drivetrain.registerTelemetry(telemetry::telemeterize);
    }
  }
  public double getJoystickLeftX() {
    return driverController.getLeftX();
  }
  public double getJoystickLeftY() {
    return driverController.getLeftY();
  }
  public double getJoystickRightX() {
    return driverController.getRightX();
  }
  public double getJoystickRightY() {
    return driverController.getRightY();
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
