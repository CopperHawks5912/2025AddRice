// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.intake.DeployArmCommand;
import frc.robot.commands.intake.EatNoteCommand;
import frc.robot.commands.intake.EatNoteWithDelayCommand;
import frc.robot.commands.intake.HomeArmCommand;
import frc.robot.commands.intake.StopIntakeCommand;
import frc.robot.commands.shooter.ShootToAmpCommand;
import frc.robot.commands.shooter.ShootToSpeakerCommand;
import frc.robot.commands.shooter.ShootToSpeakerWithDelayCommand;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.intake.IntakeArmSubsystem;
import frc.robot.subsystems.intake.IntakeGrabberSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import com.pathplanner.lib.auto.NamedCommands;

import java.io.File;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve/5912"));
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private final IntakeArmSubsystem m_IntakeArmSubsystem = new IntakeArmSubsystem();
  private final IntakeGrabberSubsystem m_IntakeGrabberSubsystem = new IntakeGrabberSubsystem();

  // CommandJoystick driverController   = new CommandJoystick(3);//(OperatorConstants.DRIVER_CONTROLLER_PORT);
  XboxController driverXbox = new XboxController(0);
  CommandGenericHID m_secondController = new CommandGenericHID(1);
  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    NamedCommands.registerCommand ("DeployArm", new DeployArmCommand(m_IntakeArmSubsystem));
    NamedCommands.registerCommand("HomeArm", new HomeArmCommand(m_IntakeArmSubsystem));
    NamedCommands.registerCommand("EatNote", new EatNoteWithDelayCommand(m_IntakeGrabberSubsystem, AutoConstants.IntakeDelaySeconds));
    NamedCommands.registerCommand("StopIntake", new StopIntakeCommand(m_IntakeGrabberSubsystem));
    NamedCommands.registerCommand("ShootToSpeakerWithDelay", new ShootToSpeakerWithDelayCommand(m_shooterSubsystem, m_IntakeGrabberSubsystem, AutoConstants.ShooterDelaySeconds ));
   
    configureBindings();

    AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
                                                                   () -> MathUtil.applyDeadband(-driverXbox.getLeftY() / 1.5,
                                                                                                ControllerConstants.LeftYDeadband),
                                                                   () -> MathUtil.applyDeadband(-driverXbox.getLeftX() / 1.5,
                                                                                                ControllerConstants.LeftXDeadband),
                                                                   () -> MathUtil.applyDeadband( driverXbox.getRightX(),
                                                                                                ControllerConstants.RightXDeadband),
                                                                   driverXbox::getYButtonPressed,
                                                                   driverXbox::getAButtonPressed,
                                                                   driverXbox::getXButtonPressed,
                                                                   driverXbox::getBButtonPressed);

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation
    
    // Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
    //     () -> MathUtil.applyDeadband(-driverXbox.getLeftY(), ControllerConstants.LeftYDeadband),
    //     () -> MathUtil.applyDeadband(-driverXbox.getLeftX(), ControllerConstants.LeftXDeadband),
    //     () -> -driverXbox.getRightX(),
    //     () -> -driverXbox.getRightY());

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    // Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
    //     () -> MathUtil.applyDeadband(-driverXbox.getLeftY(), ControllerConstants.LeftYDeadband),
    //     () -> MathUtil.applyDeadband(-driverXbox.getLeftX(), ControllerConstants.LeftXDeadband),
    //     () -> driverXbox.getRawAxis(4));

    // Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
    //     () -> MathUtil.applyDeadband(driverXbox.getLeftY(), ControllerConstants.LeftYDeadband),
    //     () -> MathUtil.applyDeadband(driverXbox.getLeftX(), ControllerConstants.LeftXDeadband),
    //     () -> driverXbox.getRawAxis(2));

    drivebase.setDefaultCommand(
        !RobotBase.isSimulation() ? closedAbsoluteDriveAdv : closedAbsoluteDriveAdv);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    m_secondController.button(ControllerConstants.ButtonBlueUpper)
        .onTrue(new DeployArmCommand(m_IntakeArmSubsystem));
    m_secondController.button(ControllerConstants.ButtonBlueLower)
        .onTrue(new HomeArmCommand(m_IntakeArmSubsystem));
    //m_secondController.button(ControllerConstants.ButtonRedUpper1)
     //   .whileTrue(new SpeakerPrelaunchCommand(m_shooterSubsystem));
    m_secondController.button(ControllerConstants.ButtonRedUpper3)
        .whileTrue(new EatNoteCommand(m_IntakeGrabberSubsystem));
    m_secondController.button(ControllerConstants.ButtonBlack1)
        .whileTrue(new ShootToAmpCommand(m_shooterSubsystem, m_IntakeGrabberSubsystem ));
    m_secondController.button(ControllerConstants.ButtonBlack2)
        .whileTrue(new ShootToSpeakerCommand(m_shooterSubsystem, m_IntakeGrabberSubsystem ));
        
    
    // new JoystickButton(driverXbox, 3).onTrue(new InstantCommand(drivebase::addFakeVisionReading));
     //new JoystickButton(driverXbox,
     //                   2).whileTrue(
     //    Commands.deferredProxy(() -> drivebase.driveToPose(
     //                               new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
     //                          ));
//    new JoystickButton(driverXbox, 3).whileTrue(new RepeatCommand(new InstantCommand(drivebase::lock, drivebase)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("Left-Note1");
    //return drivebase.getAutonomousCommand("Center-Note2").andThen(drivebase.getAutonomousCommand("Center-Note1"));
  }

  public void setDriveMode()
  {
    //drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
