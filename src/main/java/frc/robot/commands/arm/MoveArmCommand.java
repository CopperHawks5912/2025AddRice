// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.mechanisms.ArmSubsystem;

/** An example command that uses an example subsystem. */
public class MoveArmCommand extends Command {
  private final ArmSubsystem m_armSubsystem;
  private final double m_armPosition;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public MoveArmCommand(ArmSubsystem armSubsystem, double armPosition ) {
    m_armSubsystem = armSubsystem;
    m_armPosition = armPosition;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_armSubsystem.setArmPosition(m_armPosition);
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
    return m_armSubsystem.isArmAtPose();
  }
}
