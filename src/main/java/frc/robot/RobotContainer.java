// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.util.FieldUtil;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.CommandHolder;
import frc.robot.commands.PrepCommand;
import frc.robot.commands.PrepCommandForAuto;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.calibration.CalibrateWristAngleCommand;
import frc.robot.commands.intake.EjectCommand;
import frc.robot.commands.intake.Auto.MoveBack;
import frc.robot.commands.intake.Auto.NoteInShooter;
import frc.robot.commands.intake.Auto.StartIntake;
import frc.robot.generated.SwerveTunerConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.BooleanSupplier;

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

    private final CommandHolder commands;

    private SendableChooser<Command> autoChooser = new SendableChooser<Command>();

    private final ShootCommand shootCommand = new ShootCommand(shooterSubsystem, intakeSubsystem);

    private final PrepCommand stageSpeakerShoot = new PrepCommand(shooterSubsystem, 52.5, 0.9); //TODO Change angle if necessary
    private final PrepCommand trapShoot = new PrepCommand(shooterSubsystem, 66, 50); //TODO Change angle if necessary
    private final PrepCommand ampShoot = new PrepCommand(shooterSubsystem, 58, 22); //TODO Change angle if necessary
    private final PrepCommand speakerShoot = new PrepCommand(shooterSubsystem, 65, 70); //TODO Change angle if necessary
    //    private final PrepCommand longShot = new PrepCommand(shooterSubsystem, 43.5, 120); //TODO Change angle if necessary
    private final PrepCommand stopShoot = new PrepCommand(shooterSubsystem, 30, 0);
    private final EjectCommand ejectCommand = new EjectCommand(intakeSubsystem);

    private final PrepCommandForAuto autoSpeakerPrep = new PrepCommandForAuto(shooterSubsystem, 67, 80);


    private final Command intakeCommand = new StartIntake(intakeSubsystem, shooterSubsystem)
            .andThen(new NoteInShooter(intakeSubsystem, shooterSubsystem)
            .andThen(new MoveBack(intakeSubsystem, shooterSubsystem)));

    public final CommandXboxController driverController =
            new CommandXboxController(OperatorConstants.kDriverControllerPort);
    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */

    private final double maxSpeed = 6; // 6 meters per second desired top speed
    private final double maxAngularRate = 7.33478344093933; // 2 * Math.PI?

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
        commands = new CommandHolder(
                intakeSubsystem,
                shooterSubsystem,
                driverController.getHID()
        );
        configureBindings();
        intakeSubsystem.intakeInit();
        shooterSubsystem.shooterInit();

        NamedCommands.registerCommand("Speaker Shoot", autoSpeakerPrep);
        NamedCommands.registerCommand("Shoot", shootCommand);
        NamedCommands.registerCommand("Stage Speaker Shoot", stageSpeakerShoot);
        NamedCommands.registerCommand("Trap Shoot", trapShoot);
        NamedCommands.registerCommand("Intake", intakeCommand);

        // TODO: tune positions of robot especially with bumpers
        autoChooser = AutoBuilder.buildAutoChooser("");
        SmartDashboard.putData("Auto Chooser", autoChooser);

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
        // Uncomment this to calibrate the wrist angle
        shooterSubsystem.setDefaultCommand(commands.calibrateWristAngleCommand());

        intakeSubsystem.setDefaultCommand(commands.waitForIntakeCommand());

        driverController.a().toggleOnTrue(ejectCommand);  // Allow ejecting a note to be stopped on a second a press
        driverController.b().toggleOnTrue(commands.intakeNoteAndKeepRunningCommand());
        driverController.x().and(shooterSubsystem::isNoteInShooter).onTrue(trapShoot);
        driverController.y().and(shooterSubsystem::isNoteInShooter).onTrue(stageSpeakerShoot);

        driverController.leftBumper().and(shooterSubsystem::isNoteInShooter).onTrue(ampShoot);
        driverController.rightBumper().and(shooterSubsystem::isNoteInShooter).onTrue(speakerShoot);
        driverController.leftTrigger().onTrue(stopShoot);

        driverController.rightTrigger()
                .and(shooterSubsystem::canShoot)  // Don't allow shooting unless the shooter is ready
                .and(shooterSubsystem::isNoteReadyToFire)
                .onTrue(shootCommand);

        drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> {
                            double originalX = -driverController.getLeftY() * (!FieldUtil.isAllianceBlue() ? -1 : 1);
                            double originalY = -driverController.getLeftX() * (!FieldUtil.isAllianceBlue() ? -1 : 1);
                            double newX = originalX * Math.sqrt(1 - ((originalY * originalY) / 2));
                            double newY = originalY * Math.sqrt(1 - ((originalX * originalX) / 2));
                            return drive.withVelocityX(newX * maxSpeed) // Drive forward with
                                    // negative Y (forward)
                                    .withVelocityY(newY * maxSpeed) // Drive left with negative X (left)
                                    .withRotationalRate(-driverController.getRightX() * maxAngularRate); // Drive counterclockwise with negative X (left)
                        }
                ));

        driverController.back().onTrue(Commands.runOnce(() -> commands.calibrateWristAngleCommand().recalibrate()));
        driverController.start().whileTrue(drivetrain
                .applyRequest(() -> point.withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))));

        // reset the field-centric heading on start button press
        driverController.start().onTrue(drivetrain.runOnce(drivetrain::zeroGyroAdjusted));

        BooleanSupplier hasValidPrepCommand = () -> PrepCommand.currentPrepCommand != null;
        driverController.povRight().and(hasValidPrepCommand).onTrue(Commands.runOnce(() -> PrepCommand.currentPrepCommand.changeAngle(+3)));
        driverController.povLeft().and(hasValidPrepCommand).onTrue(Commands.runOnce(() -> PrepCommand.currentPrepCommand.changeAngle(-3)));

        driverController.povUp().and(hasValidPrepCommand).onTrue(Commands.runOnce(() -> PrepCommand.currentPrepCommand.changeSpeed(+5)));
        driverController.povDown().and(hasValidPrepCommand).onTrue(Commands.runOnce(() -> PrepCommand.currentPrepCommand.changeSpeed(-5)));

        if (Utils.isSimulation()) {
            drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));

            drivetrain.registerTelemetry(telemetry::telemeterize);
        }
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
