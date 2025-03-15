// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveInputStream;

/** An example command that uses an example subsystem. */
public class ReduceSwerveTranslationCommand extends Command {
  SwerveSubsystem m_swerve;
  SwerveInputStream m_inputStream;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ReduceSwerveTranslationCommand(  SwerveSubsystem swerve, SwerveInputStream inputStream) {
    m_swerve = swerve;
    m_inputStream = inputStream;  
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_inputStream.scaleTranslation( SwerveConstants.SlowModeScaleTranlastion );
    m_inputStream.scaleRotation( SwerveConstants.SlowModeScaleRotation );
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      
  }    

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    double translationScale = m_swerve.GetCurrentScaleTranslation();
    double rotationScale = m_swerve.GetCurrentScaleRotation();
    m_inputStream.scaleTranslation( translationScale );
    m_inputStream.scaleRotation( rotationScale );
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {  
    return false;
  }
}
