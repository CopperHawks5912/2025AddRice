package frc.robot.subsystems.climber;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
  private WPI_VictorSPX m_leftClimber = new WPI_VictorSPX( CANConstants.LeftClimberID );
  private WPI_VictorSPX m_rightClimber = new WPI_VictorSPX( CANConstants.RightClimberID );
 
  public ClimberSubsystem() {
    m_leftClimber.configFactoryDefault();   
    m_leftClimber.stopMotor();
    m_leftClimber.setNeutralMode( NeutralMode.Brake); 
   
    m_rightClimber.configFactoryDefault();   
    m_rightClimber.stopMotor();
    m_rightClimber.setNeutralMode( NeutralMode.Brake);      
 }

  @Override
  public void periodic() {     
  }

  public void extendClimbers()
  {
    extendClimber( m_leftClimber );
    extendClimber( m_rightClimber );
  }  
  public void retractClimbers()
  {
    retractClimber( m_leftClimber );
    retractClimber( m_rightClimber );
  }  
  public void stopClimbers()
  {
    stopClimber( m_leftClimber );
    stopClimber( m_rightClimber );
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

  public boolean areClimbersStalled()
  {
    boolean retVal = false;

    return retVal;
  }
}
