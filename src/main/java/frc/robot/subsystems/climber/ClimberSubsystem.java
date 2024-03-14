package frc.robot.subsystems.climber;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
  private WPI_TalonSRX m_leftClimber = new WPI_TalonSRX( CANConstants.LeftClimberID );
  private WPI_VictorSPX m_rightClimber = new WPI_VictorSPX( CANConstants.RightClimberID );
  
  private int m_timeoutMs = 30;

  public ClimberSubsystem() {
    m_leftClimber.configFactoryDefault();   
    m_leftClimber.stopMotor();
    m_leftClimber.setNeutralMode( NeutralMode.Brake); 
   
    m_leftClimber.setSelectedSensorPosition(0, 0, m_timeoutMs);

    m_rightClimber.configFactoryDefault();   
    m_rightClimber.stopMotor();
    m_rightClimber.setNeutralMode( NeutralMode.Brake);   
    
    m_rightClimber.follow(m_leftClimber);
    m_rightClimber.setInverted(InvertType.OpposeMaster);  
 }

  @Override
  public void periodic() {     
  }

  public void extendClimbers()
  {
    if( m_leftClimber.getSelectedSensorPosition(0) <= ClimberConstants.ExtendedPosition )
    {
      extendClimber( m_leftClimber );
    //extendClimber( m_rightClimber );
    }
    else
    {
      stopClimbers();
    }
  }  
  public void retractClimbers()
  {
    if( m_leftClimber.getSelectedSensorPosition(0) >= ClimberConstants.HomePosition )
    {
      retractClimber( m_leftClimber );
      //retractClimber( m_rightClimber );
    }
    else
    {
      stopClimbers();
    }
  }  
  public void stopClimbers()
  {
    stopClimber( m_leftClimber );
    //stopClimber( m_rightClimber );
  }  

  public void extendClimber( int climberID )
  {
      if( climberID == CANConstants.LeftClimberID )
         extendClimber( m_leftClimber );
      else if( climberID == CANConstants.RightClimberID )
         extendClimber( m_leftClimber );
  }  
  
  public void retractClimber( int climberID )
  {
      if( climberID == CANConstants.LeftClimberID )
         retractClimber( m_leftClimber );
      else if( climberID == CANConstants.RightClimberID )
         retractClimber( m_leftClimber );
  }
  public void stopClimber( int climberID )
  {
      if( climberID == CANConstants.LeftClimberID )
         stopClimber( m_leftClimber );
      else if( climberID == CANConstants.RightClimberID )
         stopClimber( m_leftClimber );
  }
  public void extendClimber( WPI_VictorSPX climber )
  {
     climber.set( VictorSPXControlMode.PercentOutput, ClimberConstants.ExtendSpeed);
  }  
  public void retractClimber(  WPI_VictorSPX climber ) 
  {
     climber.set( VictorSPXControlMode.PercentOutput, ClimberConstants.RetractSpeed);
  }  
  public void stopClimber( WPI_VictorSPX climber ) 
  {
     climber.set( VictorSPXControlMode.PercentOutput, 0 );
  } 
public void extendClimber( WPI_TalonSRX climber )
  {
     climber.set( TalonSRXControlMode.PercentOutput, ClimberConstants.ExtendSpeed);
  }  
  public void retractClimber(  WPI_TalonSRX climber ) 
  {
     climber.set( TalonSRXControlMode.PercentOutput, ClimberConstants.RetractSpeed);
  }  
  public void stopClimber( WPI_TalonSRX climber ) 
  {
     climber.set( TalonSRXControlMode.PercentOutput, 0 );
  } 

  public boolean areClimbersStalled()
  {
    boolean retVal = false;

    return retVal;
  }
}
