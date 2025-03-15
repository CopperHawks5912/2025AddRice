// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.mechanisms.ElevatorSubsystem;

/** An example command that uses an example subsystem. */
public class MoveElevatorCommand extends Command {
  private final ElevatorSubsystem m_elevatorSubsystem;
  private final double m_ElevatorPosition;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public MoveElevatorCommand(ElevatorSubsystem elevatorSubsystem, double elevatorPosition ) {
    m_elevatorSubsystem = elevatorSubsystem;
    m_ElevatorPosition = elevatorPosition;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_elevatorSubsystem.setElevatorPosition(m_ElevatorPosition);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }    

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if( m_elevatorSubsystem.isElevatorAtPose() && m_ElevatorPosition == ElevatorConstants.HomePosition )
    {
      m_elevatorSubsystem.stopElevator();
      ///m_elevatorSubsystem.setZeroPoint();  //dont actually do this, probably  
    } 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {  
    return m_elevatorSubsystem.isElevatorAtPose();
  }
}
