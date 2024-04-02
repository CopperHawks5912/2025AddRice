package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.DIOConstants;
import frc.robot.Constants.IntakeArmConstants;

public class IntakeArmSubsystem extends SubsystemBase {
  private WPI_TalonSRX m_leftArmTalon = new WPI_TalonSRX( CANConstants.LeftIntakeArmID );
  private WPI_VictorSPX m_rightArmVictor = new WPI_VictorSPX( CANConstants.RightIntakeArmID );
  private DigitalInput m_intakeArmLimitSwitch = new DigitalInput(DIOConstants.IntakeArmLimitSwitch);
  
  private static final int kTimeoutMs = 30;
  private double m_ArbitraryFeedForward;
  private int m_currentTarget;
  private double m_currentPosition;
 
  public IntakeArmSubsystem( ) {
    m_leftArmTalon.configFactoryDefault();   
    m_leftArmTalon.stopMotor();
    m_leftArmTalon.setNeutralMode( NeutralMode.Brake); 
    
    m_rightArmVictor.configFactoryDefault();   
    m_rightArmVictor.stopMotor();
    m_rightArmVictor.setNeutralMode( NeutralMode.Brake); 
    m_rightArmVictor.follow(m_leftArmTalon);
    
    /* (sample code comment)
       set deadband to super small 0.001 (0.1 %).
			 The default deadband is 0.04 (4 %) */
    m_leftArmTalon.configNeutralDeadband(0.001, kTimeoutMs);
    m_rightArmVictor.configNeutralDeadband(0.001, kTimeoutMs);
    
    /* Set relevant frame periods to be at least as fast as periodic rate */
    m_leftArmTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, kTimeoutMs);
    m_leftArmTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, kTimeoutMs);
    
    /* Set the peak and nominal outputs */
    m_leftArmTalon.configNominalOutputForward(0, kTimeoutMs);
    m_leftArmTalon.configNominalOutputReverse(0, kTimeoutMs);
		
//    m_leftArmTalon.configPeakOutputForward(IntakeArmConstants.ArmMaxPeakOutputForward, kTimeoutMs);
//	  m_leftArmTalon.configPeakOutputReverse(IntakeArmConstants.ArmMaxPeakOutputReverse, kTimeoutMs);
   
    /* Set Motion Magic gains in slot0 - see documentation */
    m_leftArmTalon.selectProfileSlot( IntakeArmConstants.PIDProfileSlotIndex, IntakeArmConstants.PIDLoopIndex);
    m_leftArmTalon.config_kF(IntakeArmConstants.PIDProfileSlotIndex, IntakeArmConstants.ArmGains.kF, kTimeoutMs);
    m_leftArmTalon.config_kP(IntakeArmConstants.PIDProfileSlotIndex, IntakeArmConstants.ArmGains.kP, kTimeoutMs);
    m_leftArmTalon.config_kI(IntakeArmConstants.PIDProfileSlotIndex, IntakeArmConstants.ArmGains.kI, kTimeoutMs);
    m_leftArmTalon.config_kD(IntakeArmConstants.PIDProfileSlotIndex, IntakeArmConstants.ArmGains.kD, kTimeoutMs);    

    /* Set acceleration and vcruise velocity - see documentation */
    m_leftArmTalon.configMotionCruiseVelocity(IntakeArmConstants.ArmCruiseVelocity, kTimeoutMs);
    m_leftArmTalon.configMotionAcceleration( IntakeArmConstants.ArmAcceleration, kTimeoutMs);
    
    
    /* Zero the sensor once on robot boot up */
    m_leftArmTalon.setSelectedSensorPosition(0, IntakeArmConstants.PIDLoopIndex, kTimeoutMs);	
    m_leftArmTalon.setSensorPhase(false);   
    m_leftArmTalon.setInverted(InvertType.None);  
    
    m_rightArmVictor.setInverted(InvertType.FollowMaster);  
      
 }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //m_leftArmTalon.setSensorPhase(false);     
 
    // SmartDashboard.putData(this);
    // SmartDashboard.putNumber( "Arm Position", m_leftArmTalon.getSelectedSensorPosition(IntakeArmConstants.PIDLoopIndex) );
    // SmartDashboard.putNumber( "Arm Power", m_leftArmTalon.getMotorOutputPercent() );
    // //SmartDashboard.putNumber( "Arm Target", m_currentTarget.getShoulderPosition() );
    // //SmartDashboard.putBoolean( "Shoulder Switch", m_shoulderLimitSwitch.get() );

     SmartDashboard.putNumber( "Arm Arb FF", m_ArbitraryFeedForward );  
     
    if( m_currentTarget == 0 && m_intakeArmLimitSwitch.get() )
    {
      m_leftArmTalon.set(0);
      m_leftArmTalon.setSelectedSensorPosition(0, IntakeArmConstants.PIDLoopIndex, kTimeoutMs);
    }  
    m_currentPosition = m_leftArmTalon.getSelectedSensorPosition(IntakeArmConstants.PIDLoopIndex);
     
     SmartDashboard.putNumber( "Arm Position", m_currentPosition );    
     SmartDashboard.putNumber( "Arm Target", m_currentTarget );      
     SmartDashboard.putNumber( "Arm Output", m_leftArmTalon.getMotorOutputPercent() );    
     SmartDashboard.putBoolean( "Arm Limit Switch", m_intakeArmLimitSwitch.get() );  
     
  }

  public void moveArmToPosition( int position )
  {
    m_currentTarget = position;    
    //double currentPos = m_leftArmTalon.getSelectedSensorPosition(IntakeArmConstants.PIDLoopIndex);    
    if( m_currentTarget == 0 && m_intakeArmLimitSwitch.get() )
    {
      m_leftArmTalon.set(0);
      m_leftArmTalon.setSelectedSensorPosition(0, IntakeArmConstants.PIDLoopIndex, kTimeoutMs);
    }  
    else  
    {
      m_ArbitraryFeedForward = calculateArbitraryFeedForward();
      m_leftArmTalon.set( ControlMode.MotionMagic, m_currentTarget, DemandType.ArbitraryFeedForward, m_ArbitraryFeedForward );
    }
  }  

  public boolean isInPosition()
  {
    if( Math.abs( m_currentPosition - m_currentTarget ) < 25 )
      return true;
    else 
      return false;
  }

  private double calculateArbitraryFeedForward( )
  {
    double kTicksPerDegree = IntakeArmConstants.EncoderCountsPerRev * IntakeArmConstants.ArmGearRatio / 360; 
    double currentPos = m_leftArmTalon.getSelectedSensorPosition();    
   
    double degrees = ( currentPos - IntakeArmConstants.ArmHorizontalPosition) / kTicksPerDegree;
    double radians = java.lang.Math.toRadians(degrees);
    double cosineScalar = java.lang.Math.cos(radians);
    
    double arbitraryFF = IntakeArmConstants.ArmMaxGravityFF * cosineScalar;
     return arbitraryFF;
  }  
}
