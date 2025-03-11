// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DIOConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ReefPoseConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.TestRumbleCommand;
import frc.robot.commands.LED.AllianceLEDCommand;
import frc.robot.commands.LED.CopperHawksLEDCommand;
import frc.robot.subsystems.LED.AddressableLEDSubsystem;
import frc.robot.subsystems.mechanisms.ArmSubsystem;
import frc.robot.subsystems.mechanisms.ElevatorSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveInputStream;

import com.pathplanner.lib.auto.NamedCommands;
import frc.robot.commands.swervedrive.drivebase.*;
import java.io.File;
import java.util.Optional;
import swervelib.SwerveInputStream;
import swervelib.simulation.SwerveModuleSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

import frc.robot.commands.mechanisms.MoveArmCommand;
import frc.robot.commands.mechanisms.MoveElevatorCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandXboxController driverXbox = new CommandXboxController(0);
  final CommandGenericHID operatorController = new CommandGenericHID(1);
  
//  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
//  private final ArmSubsystem armSubsystem = new ArmSubsystem();
  
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
                                                            .withControllerRotationAxis(driverXbox::getRightX)
                                                            .deadband(SwerveConstants.Deadband)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
                                                                                             driverXbox::getRightY)
                                                           .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                        () -> -driverXbox.getLeftY(),
                                                                        () -> -driverXbox.getLeftX())
                                                                    .withControllerRotationAxis(() -> driverXbox.getRawAxis(
                                                                        2))
                                                                    .deadband(SwerveConstants.Deadband)
                                                                    .scaleTranslation(0.8)
                                                                    .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
                                                                               .withControllerHeadingAxis(() ->
                                                                                                              Math.sin(
                                                                                                                  driverXbox.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2),
                                                                                                          () ->
                                                                                                              Math.cos(
                                                                                                                  driverXbox.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2))
                                                                               .headingWhile(true)
                                                                               .translationHeadingOffset(true)
                                                                               .translationHeadingOffset(Rotation2d.fromDegrees(
                                                                                   0));

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */

    Command driveFieldOrientedDirectAngle      = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity  = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard      = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngleKeyboard);

  public RobotContainer()
  {
//    elevatorSubsystem.setDefaultCommand ( new MoveElevatorCommand( elevatorSubsystem, ElevatorConstants.HomePosition ) );
//    armSubsystem.setDefaultCommand ( new MoveArmCommand( armSubsystem, ArmConstants.HomePosition ) );

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
    if (RobotBase.isSimulation())
    {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } else
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }
    
    // operatorController.button(ControllerConstants.ButtonBlueUpper)
    //      .whileTrue(new MoveElevatorCommand(elevatorSubsystem, ElevatorConstants.Lvl1Position) 
    //      .alongWith( new MoveArmCommand(armSubsystem, ArmConstants.Lvl1Position) ) );
    // operatorController.button(ControllerConstants.ButtonRedUpper1)
    //      .whileTrue(new MoveElevatorCommand(elevatorSubsystem, ElevatorConstants.Lvl2Position) 
    //      .alongWith( new MoveArmCommand(armSubsystem, ArmConstants.Lvl2Position) ) );
    // operatorController.button(ControllerConstants.ButtonRedUpper2)
    //      .whileTrue(new MoveElevatorCommand(elevatorSubsystem, ElevatorConstants.Lvl3Position) 
    //      .alongWith( new MoveArmCommand(armSubsystem, ArmConstants.Lvl3Position) ) );
    // operatorController.button(ControllerConstants.ButtonRedUpper3)
    //      .whileTrue(new MoveElevatorCommand(elevatorSubsystem, ElevatorConstants.Lvl4Position) 
    //      .alongWith( new WaitCommand(.5).andThen ( new MoveArmCommand(armSubsystem, ArmConstants.Lvl4Position) )  ) );
    
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

 


    if (Robot.isSimulation())
    {
      Pose2d target = new Pose2d(new Translation2d(1, 4),
                                 Rotation2d.fromDegrees(90));
      //drivebase.getSwerveDrive().field.getObject("targetPose").setPose(target);
      driveDirectAngleKeyboard.driveToPose(() -> target,
                                           new ProfiledPIDController(5,
                                                                     0,
                                                                     0,
                                                                     new Constraints(5, 2)),
                                           new ProfiledPIDController(5,
                                                                     0,
                                                                     0,
                                                                     new Constraints(Units.degreesToRadians(360),
                                                                                     Units.degreesToRadians(180))
                                           ));
      driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
      driverXbox.button(2).whileTrue(Commands.runEnd(() -> driveDirectAngleKeyboard.driveToPoseEnabled(true),
                                                     () -> driveDirectAngleKeyboard.driveToPoseEnabled(false))); 

    }
    if (DriverStation.isTest())
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

      driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.back().whileTrue(drivebase.centerModulesCommand());
      driverXbox.x().whileTrue(drivebase.driveToReefPosition(ReefPoseConstants.ScoringAlignment.LEFT));
      driverXbox.y().whileTrue(drivebase.driveToReefPosition(ReefPoseConstants.ScoringAlignment.CENTER));
      driverXbox.b().whileTrue(drivebase.driveToReefPosition(ReefPoseConstants.ScoringAlignment.RIGHT));
      driverXbox.rightBumper().onTrue(Commands.none());
    } else
    {
      driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      driverXbox.start().whileTrue(Commands.none());
      driverXbox.back().whileTrue(Commands.none());
      driverXbox.x().whileTrue(drivebase.driveToReefPosition(ReefPoseConstants.ScoringAlignment.LEFT));
      driverXbox.y().whileTrue(drivebase.driveToReefPosition(ReefPoseConstants.ScoringAlignment.CENTER));
      driverXbox.b().whileTrue(drivebase.driveToReefPosition(ReefPoseConstants.ScoringAlignment.RIGHT));
      driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.rightBumper().onTrue(Commands.none());
    }

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
