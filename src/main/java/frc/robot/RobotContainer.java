// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DIOConstants;
import frc.robot.commands.TestRumbleCommand;
import frc.robot.commands.LED.AllianceLEDCommand;
import frc.robot.commands.LED.CopperHawksLEDCommand;
import frc.robot.commands.LED.NoteLEDCommand;
import frc.robot.commands.LED.ShootingLEDCommand;
import frc.robot.commands.climber.ClimbCommand;
import frc.robot.commands.intake.ExtendArmCommand;
import frc.robot.commands.intake.EatNoteCommand;
import frc.robot.commands.intake.EatNoteWithDelayCommand;
import frc.robot.commands.intake.RetractArmCommand;
import frc.robot.commands.intake.SpitNoteCommand;
import frc.robot.commands.intake.StopIntakeCommand;
import frc.robot.subsystems.LED.AddressableLEDSubsystem;
import frc.robot.commands.shooter.ShootToAmpCommand;
import frc.robot.commands.shooter.ShootToSpeakerCommand;
import frc.robot.commands.shooter.ShootToSpeakerWithDelayCommand;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.intake.IntakeArmSubsystem;
import frc.robot.subsystems.intake.IntakeGrabberSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import com.pathplanner.lib.auto.NamedCommands;

import java.io.File;
import java.util.Optional;

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
  private DigitalInput m_intakeNoteBeamBreakSensor = new DigitalInput(DIOConstants.IntakeNoteBeamBreakSensorPort);
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private final IntakeArmSubsystem m_IntakeArmSubsystem = new IntakeArmSubsystem( );
  private final IntakeGrabberSubsystem m_IntakeGrabberSubsystem = new IntakeGrabberSubsystem( m_intakeNoteBeamBreakSensor );
  private final ClimberSubsystem m_ClimberSubsystem = new ClimberSubsystem();
  private final AddressableLEDSubsystem m_addressableLEDSubsystem = new AddressableLEDSubsystem();
  Trigger intakeTrigger = new Trigger(m_intakeNoteBeamBreakSensor::get );
    
  XboxController m_driverXboxController = new XboxController(0);
  CommandGenericHID m_operatorController = new CommandGenericHID(1);
  

  private final SendableChooser<String> m_autoDelayChooser = new SendableChooser<>();
  private final SendableChooser<String> m_autoPathChooser = new SendableChooser<>();
  private String m_selectedDelayAuto;
  private String m_selectedPathAuto;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    m_addressableLEDSubsystem.setDefaultCommand(new CopperHawksLEDCommand(m_addressableLEDSubsystem).ignoringDisable(true));
    configureAutos();
    configureBindings();

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
     

     Command driveTriggerRotate = drivebase.driveTriggerRotate(
         () -> MathUtil.applyDeadband(-m_driverXboxController.getLeftY(), ControllerConstants.LeftYDeadband),
         () -> MathUtil.applyDeadband(-m_driverXboxController.getRightX(), ControllerConstants.LeftXDeadband),
         () -> m_driverXboxController.getRawAxis(2), 
         () -> m_driverXboxController.getRawAxis(3));

    // Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
    //     () -> MathUtil.applyDeadband(driverXbox.getLeftY(), ControllerConstants.LeftYDeadband),
    //     () -> MathUtil.applyDeadband(driverXbox.getLeftX(), ControllerConstants.LeftXDeadband),
    //     () -> driverXbox.getRawAxis(2));

    
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
    //climbCommand current uses ButtonRedUpper1, ButtonRedLower1, ButtonRedUpper2, ButtonRedLower2;
    //we're passing in the driver controller so we could potentially make it rumble.
    m_ClimberSubsystem.setDefaultCommand(new ClimbCommand(m_ClimberSubsystem, m_operatorController));

    new JoystickButton(m_driverXboxController, 8).onTrue((new InstantCommand(drivebase::zeroGyro)));
    
    new JoystickButton(m_driverXboxController, 1).onTrue((new NoteLEDCommand(m_addressableLEDSubsystem)));
    new JoystickButton(m_driverXboxController, 2).onTrue((new ShootingLEDCommand(m_addressableLEDSubsystem)));
    //new JoystickButton(m_driverXboxController, 1).whileTrue(new TestRumbleCommand( m_driverXboxController ));
    
    
    // new JoystickButton(m_driverXboxController, 7)
    //    .onTrue( new InstantCommand( return run(() -> {} drivebase.setMaximumSpeed( 8.0) ) ) );

    //    .onFalse( new InstantCommand(drivebase->setMaximumSpeed( 14.5) ) );

    m_operatorController.button(ControllerConstants.ButtonBlueUpper)
        .onTrue(new ExtendArmCommand(m_IntakeArmSubsystem));
    m_operatorController.button(ControllerConstants.ButtonBlueLower)
        .onTrue( new RetractArmCommand(m_IntakeArmSubsystem) );
    m_operatorController.button(ControllerConstants.ButtonRedUpper3)
        .whileTrue(new EatNoteCommand(m_IntakeGrabberSubsystem) );
    m_operatorController.button(ControllerConstants.ButtonRedLower3)
        .whileTrue(new SpitNoteCommand(m_IntakeGrabberSubsystem));
    m_operatorController.button(ControllerConstants.ButtonBlack1)
        .whileTrue(new ShootToAmpCommand(m_shooterSubsystem, m_IntakeGrabberSubsystem ));
    m_operatorController.button(ControllerConstants.ButtonBlack2)
        .whileTrue(new ShootingLEDCommand(m_addressableLEDSubsystem).andThen(new ShootToSpeakerCommand(m_shooterSubsystem, m_IntakeGrabberSubsystem )))
        .onFalse(new AllianceLEDCommand(m_addressableLEDSubsystem));
        
    
    // new JoystickButton(driverXbox, 3).onTrue(new InstantCommand(drivebase::addFakeVisionReading));
     //new JoystickButton(driverXbox,
     //                   2).whileTrue(
     //    Commands.deferredProxy(() -> drivebase.driveToPose(
     //                               new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
     //                          ));
//    new JoystickButton(driverXbox, 3).whileTrue(new RepeatCommand(new InstantCommand(drivebase::lock, drivebase)));
  }

  private void configureAutos()
  {
    NamedCommands.registerCommand ("DeployArm", new ExtendArmCommand(m_IntakeArmSubsystem));
    NamedCommands.registerCommand("HomeArm", new RetractArmCommand(m_IntakeArmSubsystem));
    NamedCommands.registerCommand("EatNote", new EatNoteWithDelayCommand(m_IntakeGrabberSubsystem, AutoConstants.IntakeDelaySeconds));
    NamedCommands.registerCommand("StopIntake", new StopIntakeCommand(m_IntakeGrabberSubsystem));
    NamedCommands.registerCommand("ShootToSpeakerWithDelay", new ShootToSpeakerWithDelayCommand(m_shooterSubsystem, m_IntakeGrabberSubsystem, AutoConstants.ShooterDelaySeconds ));
    NamedCommands.registerCommand("ShootToSpeakerWithFastDelay", new ShootToSpeakerWithDelayCommand(m_shooterSubsystem, m_IntakeGrabberSubsystem, AutoConstants.ShooterFastDelaySeconds ));
   

    m_autoDelayChooser.setDefaultOption( "0 Sec Delay", "0");
    m_autoDelayChooser.addOption( "1 Sec Delay", "1");
    m_autoDelayChooser.addOption( "2 Sec Delay", "2");
    m_autoDelayChooser.addOption( "3 Sec Delay", "3");
    m_autoDelayChooser.addOption( "5 Sec Delay", "5");
    
    m_autoPathChooser.setDefaultOption( "Any Pre-loaded Only", "P");
    m_autoPathChooser.addOption( "Center M", "C-M");
    m_autoPathChooser.addOption( "Center M-A", "C-MA");
    m_autoPathChooser.addOption( "Center M-S", "C-MS");
    m_autoPathChooser.addOption( "AmpSide A", "A-A");
    m_autoPathChooser.addOption( "StageSide Move", "S-Mv");
    m_autoPathChooser.addOption( "None", "N");
    
    SmartDashboard.putData("Auto-Delay:", m_autoDelayChooser );
    SmartDashboard.putData("Auto-Drive:", m_autoPathChooser );

  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    Command delayCommand = null;
    Command pathCommand = null;
    m_selectedDelayAuto = m_autoDelayChooser.getSelected();
    m_selectedPathAuto = m_autoPathChooser.getSelected();

    switch( m_selectedDelayAuto )
    {
      case "1":
        delayCommand = new WaitCommand(1);
        break;
      case "2":
        delayCommand = new WaitCommand(2);
        break;
      case "3":
        delayCommand = new WaitCommand(3);
        break; 
      case "4":
        delayCommand = new WaitCommand(4);
        break;   
      case "5":
        delayCommand = new WaitCommand(5);
        break;   
      case "10":
        delayCommand = new WaitCommand(10);
        break;   
      
    }
    switch( m_selectedPathAuto )
    {  
      case "P":
        pathCommand = new ShootToSpeakerWithDelayCommand(m_shooterSubsystem, m_IntakeGrabberSubsystem, AutoConstants.ShooterDelaySeconds );
        break;
      case "C-M":
        pathCommand =  drivebase.getAutonomousCommand("Center-Note2");
        break;
      case "C-MA":
        pathCommand =  drivebase.getAutonomousCommand("Center-Note2")
                                  .andThen(drivebase.getAutonomousCommand("Center-Note1"));
        break;
      case "C-MS":
        pathCommand =  drivebase.getAutonomousCommand("Center-Note2")
                                  .andThen(drivebase.getAutonomousCommand("Center-Note3"));
        break;
      case "C-A":
        pathCommand =  drivebase.getAutonomousCommand("Center-Note1");
        break;
      case "A-A":
        pathCommand =  drivebase.getAutonomousCommand("Left-Note1");
        break;
      case "S-Mv":
        pathCommand =  drivebase.getAutonomousCommand("Right-Movement");
        break;
    }

    if( delayCommand != null && pathCommand != null)
      return delayCommand.andThen(pathCommand);
    else if( pathCommand != null )
      return pathCommand;
    else
      return null;
  

    // An example command will be run in autonomous
    //return drivebase.getAutonomousCommand("CenterTest");
    //return drivebase.getAutonomousCommand("Center-Note2").andThen(drivebase.getAutonomousCommand("Center-Note1"));
  }

  public void enableIntakeTrigger()
  {    
    intakeTrigger.onFalse( new ParallelDeadlineGroup( 
                                    new RetractArmCommand(m_IntakeArmSubsystem), 
                                    new NoteLEDCommand( m_addressableLEDSubsystem ), 
                                    new TestRumbleCommand( m_driverXboxController ))
                      .andThen( new AllianceLEDCommand( m_addressableLEDSubsystem )));
  }
  // public void disableIntakeTrigger()
  // {    
  //   intakeTrigger.onFalse( null);
  // }
  public void setDriveMode()
  {
    m_addressableLEDSubsystem.setDefaultCommand(new AllianceLEDCommand(m_addressableLEDSubsystem));
    
    Command driveCommand;
    
    AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
                                                                   () -> MathUtil.applyDeadband(-m_driverXboxController.getLeftY(),
                                                                                                ControllerConstants.LeftYDeadband),
                                                                   () -> MathUtil.applyDeadband(-m_driverXboxController.getLeftX(),
                                                                                                ControllerConstants.LeftXDeadband),
                                                                   () -> MathUtil.applyDeadband( m_driverXboxController.getRightX(),
                                                                                                ControllerConstants.RightXDeadband),
                                                                   m_driverXboxController::getYButtonPressed,
                                                                   m_driverXboxController::getAButtonPressed,
                                                                   m_driverXboxController::getXButtonPressed,
                                                                   m_driverXboxController::getBButtonPressed);

    Optional<Alliance>  alliance = DriverStation.getAlliance();
    if( alliance.get() == Alliance.Blue)
    {
      driveCommand = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(-m_driverXboxController.getLeftY() * 0.85, ControllerConstants.LeftYDeadband),
        () -> MathUtil.applyDeadband(-m_driverXboxController.getLeftX() * 0.85, ControllerConstants.LeftXDeadband),
        () -> -m_driverXboxController.getRawAxis(4)* 0.85);
    }
    else
    {
      driveCommand = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(m_driverXboxController.getLeftY() * 0.85, ControllerConstants.LeftYDeadband),
        () -> MathUtil.applyDeadband(m_driverXboxController.getLeftX() * 0.85, ControllerConstants.LeftXDeadband),
        () -> -m_driverXboxController.getRawAxis(4) * 0.80);
    }
    drivebase.setDefaultCommand(
        !RobotBase.isSimulation() ? driveCommand : driveCommand);
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
