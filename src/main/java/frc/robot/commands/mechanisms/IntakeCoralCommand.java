// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.mechanisms;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.mechanisms.RollerSubsystem;

/** An example command that uses an example subsystem. */
public class IntakeCoralCommand extends Command {
  private final RollerSubsystem m_rollerSubsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public IntakeCoralCommand(RollerSubsystem rollerSubsystem) {
    m_rollerSubsystem = rollerSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(rollerSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if( !m_rollerSubsystem.isBeamBreakBroke() )
      m_rollerSubsystem.intakeCoral();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      
  }    

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_rollerSubsystem.stopRoller();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {  
    return m_rollerSubsystem.isBeamBreakBroke();
  }
}
