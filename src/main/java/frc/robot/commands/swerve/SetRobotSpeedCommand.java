// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveInputStream;

/** An example command that uses an example subsystem. */
public class SetRobotSpeedCommand extends Command {
  SwerveSubsystem m_swerve;
  SwerveInputStream m_inputStream;
  double m_translationScale;
  double m_rotationScale;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public SetRobotSpeedCommand( SwerveSubsystem swerve, SwerveInputStream inputStream, double translationScale, double rotationScale ) {
    m_swerve = swerve;
    m_inputStream = inputStream;  
    m_translationScale = translationScale;
    m_rotationScale = rotationScale;
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_swerve.setRobotSpeed(m_inputStream, m_translationScale, m_rotationScale);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      
  }    

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {  
    return true;
  }
}
