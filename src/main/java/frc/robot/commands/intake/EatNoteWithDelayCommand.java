// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeGrabberSubsystem;

/** An example command that uses an example subsystem. */
public class EatNoteWithDelayCommand extends Command {
  private final IntakeGrabberSubsystem m_IntakeGrabberSubsystem;
  private final double m_delayAfterEating;
  private Timer m_finishedEatingTimer;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public EatNoteWithDelayCommand(IntakeGrabberSubsystem intakeGrabberSubsystem, double delayAfterEating) { 
    m_IntakeGrabberSubsystem = intakeGrabberSubsystem;

    m_delayAfterEating = delayAfterEating;

    m_finishedEatingTimer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeGrabberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
     m_finishedEatingTimer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_IntakeGrabberSubsystem.intake();
    m_finishedEatingTimer.start();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_IntakeGrabberSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
  
    if (m_finishedEatingTimer.hasElapsed(m_delayAfterEating))
      return true;
    else
      return false;
  }
}
