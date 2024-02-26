// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeGrabberSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

/** An example command that uses an example subsystem. */
public class ShootToAmpCommand extends Command {
  private final ShooterSubsystem m_ShooterSubsystem;
  private final IntakeGrabberSubsystem m_IntakeGrabberSubsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ShootToAmpCommand(ShooterSubsystem shooterSubsystem, IntakeGrabberSubsystem intakeGrabberSubsystem) {
    m_ShooterSubsystem = shooterSubsystem;
    m_IntakeGrabberSubsystem = intakeGrabberSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem, intakeGrabberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ShooterSubsystem.ampShoot();
    if( m_ShooterSubsystem.isAtAmpSpeed() )
      m_IntakeGrabberSubsystem.feedShooter();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

      m_ShooterSubsystem.stopShooter();
      m_IntakeGrabberSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return false;
  }
}
