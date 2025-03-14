// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.Controller1Constants;
import frc.robot.Constants.Controller2Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.RumbleCommand;
import frc.robot.subsystems.mechanisms.ArmSubsystem;
import frc.robot.subsystems.mechanisms.ElevatorSubsystem;
import frc.robot.subsystems.mechanisms.RollerSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveInputStream;

import com.pathplanner.lib.auto.NamedCommands;
import java.io.File;

import frc.robot.commands.mechanisms.IntakeAlgaeCommand;
import frc.robot.commands.mechanisms.IntakeCoralCommand;
import frc.robot.commands.mechanisms.MoveArmCommand;
import frc.robot.commands.mechanisms.MoveElevatorCommand;
import frc.robot.commands.mechanisms.OutputAlgaeCommand;
import frc.robot.commands.mechanisms.OutputCoralCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandXboxController driverXbox = new CommandXboxController(0);
  final CommandGenericHID operatorController1 = new CommandGenericHID(1);
  final CommandGenericHID operatorController2 = new CommandGenericHID(2);
  
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private final ArmSubsystem armSubsystem = new ArmSubsystem();
  private final RollerSubsystem rollerSubsystem = new RollerSubsystem();
   
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve/5912_2025"));
//  private DigitalInput m_intakeNoteBeamBreakSensor = new DigitalInput(DIOConstants.BeamBreakSensorPort);
//  private final AddressableLEDSubsystem m_addressableLEDSubsystem = new AddressableLEDSubsystem();
//  Trigger intakeTrigger = new Trigger(m_intakeNoteBeamBreakSensor::get );
    
  private final SendableChooser<String> m_autoDelayChooser = new SendableChooser<>();
  private final SendableChooser<String> m_autoPathChooser = new SendableChooser<>();
  private String m_selectedDelayAuto;
  private String m_selectedPathAuto;


  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverXbox.getLeftY() * -1,
                                                                () -> driverXbox.getLeftX() * -1)
                                                            .withControllerRotationAxis(() -> driverXbox.getRightX() * -1)
                                                            .deadband(SwerveConstants.Deadband)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */

  Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
  
  public RobotContainer()
  {
//don't add this back in.    elevatorSubsystem.setDefaultCommand ( new MoveElevatorCommand( elevatorSubsystem, ElevatorConstants.HomePosition ) );
//don't add this back in.    armSubsystem.setDefaultCommand ( new MoveArmCommand( armSubsystem, ArmConstants.HomePosition ) );

//    m_addressableLEDSubsystem.setDefaultCommand(new CopperHawksLEDCommand(m_addressableLEDSubsystem).ignoringDisable(true));
    configureAutos();
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
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
    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

    //red buttons
    operatorController2.button(Controller2Constants.ButtonRed1)
           .onTrue( new MoveArmCommand(armSubsystem, ArmConstants.CoralMovingPosition )
           .andThen( new MoveElevatorCommand(elevatorSubsystem, ElevatorConstants.HomePosition ) )
           .andThen( new MoveArmCommand(armSubsystem, ArmConstants.HomePosition) ) );
    operatorController2.button(Controller2Constants.ButtonRed2)
           .onTrue( new MoveArmCommand(armSubsystem, ArmConstants.CoralMovingPosition )
           .andThen( new MoveElevatorCommand(elevatorSubsystem, ElevatorConstants.Lvl1Position ) )
           .andThen( new MoveArmCommand(armSubsystem, ArmConstants.Lvl1Position) ) );
    operatorController2.button(Controller2Constants.ButtonRed3)
           .onTrue( new MoveArmCommand(armSubsystem, ArmConstants.CoralMovingPosition )
           .andThen( new MoveElevatorCommand(elevatorSubsystem, ElevatorConstants.Lvl2Position ) )
           .andThen( new MoveArmCommand(armSubsystem, ArmConstants.Lvl2Position) ) );
    operatorController2.button(Controller2Constants.ButtonRed4)
           .onTrue( new MoveArmCommand(armSubsystem, ArmConstants.CoralMovingPosition )
           .andThen( new MoveElevatorCommand(elevatorSubsystem, ElevatorConstants.Lvl3Position ) )
           .andThen( new MoveArmCommand(armSubsystem, ArmConstants.Lvl3Position) ) );
    operatorController2.button(Controller2Constants.ButtonRed5)
           .onTrue( new MoveArmCommand(armSubsystem, ArmConstants.CoralMovingPosition )
           .andThen( new MoveElevatorCommand(elevatorSubsystem, ElevatorConstants.Lvl4Position ) )
           .andThen( new MoveArmCommand(armSubsystem, ArmConstants.Lvl4Position) ) );
    
    //blue buttons
   operatorController1.button(Controller1Constants.ButtonBlue1)
           .onTrue( new MoveArmCommand(armSubsystem, ArmConstants.AlgaeMovingPosition )
           .andThen( new MoveElevatorCommand(elevatorSubsystem, ElevatorConstants.HomePosition ) )
           .andThen( new MoveArmCommand(armSubsystem, ArmConstants.ProcessorAlgaePosition) ) );
    operatorController2.button(Controller2Constants.ButtonBlue2 )
           .onTrue( new MoveArmCommand(armSubsystem, ArmConstants.AlgaeMovingPosition )
           .andThen( new MoveElevatorCommand(elevatorSubsystem, ElevatorConstants.LowerAlgaePosition ) )
           .andThen( new MoveArmCommand(armSubsystem, ArmConstants.LowerAlgaePosition) ) );
    operatorController2.button(Controller2Constants.ButtonBlue3 )
           .onTrue( new MoveArmCommand(armSubsystem, ArmConstants.AlgaeMovingPosition )
           .andThen( new MoveElevatorCommand(elevatorSubsystem, ElevatorConstants.UpperAlgaePosition ) )
           .andThen( new MoveArmCommand(armSubsystem, ArmConstants.UpperAlgaePosition) ) );
    operatorController2.button(Controller2Constants.ButtonBlue4 )
           .onTrue( new MoveArmCommand(armSubsystem, ArmConstants.AlgaeMovingPosition )
           .andThen( new MoveElevatorCommand(elevatorSubsystem, ElevatorConstants.NetAlgaePosition ) )
           .andThen( new MoveArmCommand(armSubsystem, ArmConstants.NetAlgaePosition) ) );
    
    //other buttons
    operatorController1.button(Controller1Constants.ButtonYellow)
           .whileTrue( new IntakeCoralCommand(rollerSubsystem ) );
    operatorController1.button(Controller1Constants.ButtonGreen)
           .whileTrue( new OutputCoralCommand(rollerSubsystem ) );
    operatorController1.button(Controller1Constants.ButtonPlayer1)
           .whileTrue( new IntakeAlgaeCommand(rollerSubsystem ) );
    operatorController1.button(Controller1Constants.ButtonPlayer2)
           .whileTrue( new OutputAlgaeCommand(rollerSubsystem ) );
    
    driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
    driverXbox.back().onTrue(Commands.none());
    driverXbox.a().onTrue( new MoveArmCommand(armSubsystem, ArmConstants.HomePosition ) );
    driverXbox.b().onTrue(Commands.none());
    driverXbox.x().onTrue( new MoveArmCommand(armSubsystem, ArmConstants.CoralMovingPosition ) );
    driverXbox.y().onTrue(Commands.none());
    
    driverXbox.rightBumper().onTrue(Commands.none());
    driverXbox.leftBumper().onTrue(Commands.none());
  }

  private void configureAutos()
  {
    //NamedCommands.registerCommand ("DeployArm", new ExtendArmCommand(m_IntakeArmSubsystem));

    // m_autoDelayChooser.setDefaultOption( "0 Sec Delay", "0");
    // m_autoDelayChooser.addOption( "1 Sec Delay", "1");
    // m_autoDelayChooser.addOption( "2 Sec Delay", "2");
    // m_autoDelayChooser.addOption( "3 Sec Delay", "3");
    // m_autoDelayChooser.addOption( "5 Sec Delay", "5");
    
    // m_autoPathChooser.setDefaultOption( "Any Pre-loaded Only", "P");
    // m_autoPathChooser.addOption( "Center M", "C-M");
    // m_autoPathChooser.addOption( "Center M-A", "C-MA");
    // m_autoPathChooser.addOption( "Center M-S", "C-MS");
    // m_autoPathChooser.addOption( "AmpSide A", "A-A");
    // m_autoPathChooser.addOption( "StageSide Move", "S-Mv");
    // m_autoPathChooser.addOption( "StageSide Disruptor", "S-Ds");
    // m_autoPathChooser.addOption( "None", "N");
    
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
     
    
    // Command delayCommand = null;
    // Command pathCommand = null;
    // m_selectedDelayAuto = m_autoDelayChooser.getSelected();
    // m_selectedPathAuto = m_autoPathChooser.getSelected();

    // switch( m_selectedDelayAuto )
    // {
      // case "1":
      //   delayCommand = new WaitCommand(1);
      //   break;
      // case "2":
      //   delayCommand = new WaitCommand(2);
      //   break;
      // case "3":
      //   delayCommand = new WaitCommand(3);
      //   break; 
      // case "4":
      //   delayCommand = new WaitCommand(4);
      //   break;   
      // case "5":
      //   delayCommand = new WaitCommand(5);
      //   break;   
      // case "10":
      //   delayCommand = new WaitCommand(10);
      //   break;   
      
    // }
    // switch( m_selectedPathAuto )
    // {  
      // case "P":
      //   pathCommand = new ShootToSpeakerWithDelayCommand(m_shooterSubsystem, m_IntakeGrabberSubsystem, AutoConstants.ShooterDelaySeconds );
      //   break;
      // case "C-M":
      //   pathCommand =  drivebase.getAutonomousCommand("Center-Note2");
      //   break;
      // case "C-MA":
      //   pathCommand =  drivebase.getAutonomousCommand("Center-Note2")
      //                             .andThen(drivebase.getAutonomousCommand("Center-Note1"));
      //   break;
      // case "C-MS":
      //   pathCommand =  drivebase.getAutonomousCommand("Center-Note2")
      //                             .andThen(drivebase.getAutonomousCommand("Center-Note3"));
      //   break;
      // case "C-A":
      //   pathCommand =  drivebase.getAutonomousCommand("Center-Note1");
      //   break;
      // case "A-A":
      //   pathCommand =  drivebase.getAutonomousCommand("Left-Note1");
      //   break;
      // case "S-Mv":
      //   pathCommand =  drivebase.getAutonomousCommand("Right-Movement");
      //   break;
      // case "S-Ds":
      //   pathCommand =  drivebase.getAutonomousCommand("Right-Disruptor");
      //   break;
    //}

    // if( delayCommand != null && pathCommand != null)
    //   return delayCommand.andThen(pathCommand);
    // else if( pathCommand != null )
    //   return pathCommand;
    // else
       return drivebase.getAutonomousCommand("Test");
  }
  public void setDriveMode()
  {
    //m_addressableLEDSubsystem.setDefaultCommand(new AllianceLEDCommand(m_addressableLEDSubsystem));
    
    configureBindings();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
