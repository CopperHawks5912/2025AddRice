// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeGrabberSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

/** An example command that uses an example subsystem. */
public class ShootToSpeakerWithDelayCommand extends Command {
  private final ShooterSubsystem m_ShooterSubsystem;
  private final IntakeGrabberSubsystem m_IntakeGrabberSubsystem;
  private Timer m_finishedShootingTimer;
  private final double m_delayAfterShooting;
  private boolean m_isAtShootSpeed;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ShootToSpeakerWithDelayCommand(ShooterSubsystem shooterSubsystem, 
                                        IntakeGrabberSubsystem intakeGrabberSubsystem,
                                        double delayAfterShooting) {
    m_ShooterSubsystem = shooterSubsystem;
    m_IntakeGrabberSubsystem = intakeGrabberSubsystem;
    m_delayAfterShooting = delayAfterShooting;

    m_finishedShootingTimer = new Timer();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem, intakeGrabberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_isAtShootSpeed = false;
    m_finishedShootingTimer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ShooterSubsystem.speakerShoot();
    if( m_ShooterSubsystem.isAtSpeakerSpeed() )
    {
      if( !m_isAtShootSpeed )
      {
        m_IntakeGrabberSubsystem.feedShooter( false );        
        m_finishedShootingTimer.start();
      }
      m_isAtShootSpeed = true;
    
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

      m_ShooterSubsystem.stopShooter();
      m_IntakeGrabberSubsystem.stop();
      m_finishedShootingTimer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_finishedShootingTimer.hasElapsed(m_delayAfterShooting))
    {
      m_IntakeGrabberSubsystem.stop();      
      return true;
    }
    else
      return false;
  }
}
