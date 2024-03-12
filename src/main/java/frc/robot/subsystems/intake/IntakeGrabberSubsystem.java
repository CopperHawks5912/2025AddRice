package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.DIOConstants;
import frc.robot.Constants.IntakeGrabberConstants;

public class IntakeGrabberSubsystem extends SubsystemBase {
  private WPI_VictorSPX m_grabberVictor = new WPI_VictorSPX(CANConstants.IntakeGrabberID);
  private int kTimeoutMs = 30;
  private DigitalInput m_intakeLimitSwitch = new DigitalInput(DIOConstants.IntakeLimitSwitchPort);
  private boolean m_gotNote;

  public IntakeGrabberSubsystem() { 
    m_grabberVictor.configFactoryDefault();
    m_grabberVictor.stopMotor();
    m_grabberVictor.setNeutralMode( NeutralMode.Brake);   
    //m_grabberVictor.setInverted(InvertType.InvertMotorOutput);
  
    m_grabberVictor.configNeutralDeadband(0.001, kTimeoutMs);
  }

  public void intake()
  {
    //if( m_intakeLimitSwitch.get()  ) 
    //{
    //  m_grabberVictor.set(0);
    //  m_gotNote = true;
   // }
    //else
    {      
      m_grabberVictor.set( IntakeGrabberConstants.InputSpeed );
      m_gotNote = false;
    }
  }  
  public void feedShooter()
  {
    m_grabberVictor.set( IntakeGrabberConstants.OutputSpeed );
  }  
  public void stop()
  {
    m_grabberVictor.set( 0 );
  } 

  public boolean getGotNote()
  {
    return m_gotNote;
  }
}
