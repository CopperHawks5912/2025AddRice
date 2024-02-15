package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.IntakeArmConstants;
import frc.robot.Constants.IntakeGrabberConstants;

public class IntakeGrabberSubsystem extends SubsystemBase {
  private WPI_VictorSPX m_grabberVictor = new WPI_VictorSPX(CANConstants.IntakeGrabberID);
  private int kTimeoutMs = 30;

  public IntakeGrabberSubsystem() { 
    m_grabberVictor.configFactoryDefault();
    m_grabberVictor.stopMotor();
    m_grabberVictor.setNeutralMode( NeutralMode.Brake);   
    //m_grabberVictor.setInverted(InvertType.InvertMotorOutput);
  
    m_grabberVictor.configNeutralDeadband(0.001, kTimeoutMs);
  }

  public void intake()
  {
    m_grabberVictor.set( IntakeGrabberConstants.InputSpeed );
  }  
  public void feedShooter()
  {
    m_grabberVictor.set( IntakeGrabberConstants.OutputSpeed );
  }  
  public void stop()
  {
    m_grabberVictor.set( 0 );
  } 
}
