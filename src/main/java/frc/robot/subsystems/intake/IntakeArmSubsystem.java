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
import frc.robot.Constants.IntakeArmConstants;

public class IntakeArmSubsystem extends SubsystemBase {
  private WPI_TalonSRX m_armTalon = new WPI_TalonSRX( CANConstants.IntakeArmID );
  private static final int kTimeoutMs = 30;
  private double m_ArbitraryFeedForward;
  private int m_currentTarget;
 
  public IntakeArmSubsystem() {
    m_armTalon.configFactoryDefault();   
    m_armTalon.stopMotor();
    m_armTalon.setNeutralMode( NeutralMode.Brake); 
    //m_armTalon.setInverted(InvertType.None);    
 
    /* (sample code comment)
       set deadband to super small 0.001 (0.1 %).
			 The default deadband is 0.04 (4 %) */
    m_armTalon.configNeutralDeadband(0.001, kTimeoutMs);
    
    /* Set relevant frame periods to be at least as fast as periodic rate */
    m_armTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, kTimeoutMs);
    m_armTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, kTimeoutMs);
    
    /* Set the peak and nominal outputs */
    m_armTalon.configNominalOutputForward(0, kTimeoutMs);
    m_armTalon.configNominalOutputReverse(0, kTimeoutMs);
		
    m_armTalon.configPeakOutputForward(IntakeArmConstants.kArmMaxPeakOutputForward, kTimeoutMs);
	  m_armTalon.configPeakOutputReverse(IntakeArmConstants.kArmMaxPeakOutputReverse, kTimeoutMs);
   
    /* Set Motion Magic gains in slot0 - see documentation */
    m_armTalon.selectProfileSlot( IntakeArmConstants.kPIDProfileSlotIndex, IntakeArmConstants.kPIDLoopIndex);
    m_armTalon.config_kF(IntakeArmConstants.kPIDProfileSlotIndex, IntakeArmConstants.ArmGains.kF, kTimeoutMs);
    m_armTalon.config_kP(IntakeArmConstants.kPIDProfileSlotIndex, IntakeArmConstants.ArmGains.kP, kTimeoutMs);
    m_armTalon.config_kI(IntakeArmConstants.kPIDProfileSlotIndex, IntakeArmConstants.ArmGains.kI, kTimeoutMs);
    m_armTalon.config_kD(IntakeArmConstants.kPIDProfileSlotIndex, IntakeArmConstants.ArmGains.kD, kTimeoutMs);    

    /* Set acceleration and vcruise velocity - see documentation */
    m_armTalon.configMotionCruiseVelocity(IntakeArmConstants.kArmCruiseVelocity, kTimeoutMs);
    m_armTalon.configMotionAcceleration( IntakeArmConstants.kArmAcceleration, kTimeoutMs);
    
    /* Zero the sensor once on robot boot up */
    m_armTalon.setSelectedSensorPosition(0, IntakeArmConstants.kPIDLoopIndex, kTimeoutMs);	
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putData(this);
    SmartDashboard.putNumber( "Arm Position", m_armTalon.getSelectedSensorPosition(IntakeArmConstants.kPIDLoopIndex) );
    SmartDashboard.putNumber( "Arm Power", m_armTalon.getMotorOutputPercent() );
    //SmartDashboard.putNumber( "Arm Target", m_currentTarget.getShoulderPosition() );
    //SmartDashboard.putBoolean( "Shoulder Switch", m_shoulderLimitSwitch.get() );

    m_ArbitraryFeedForward = calculateArbitraryFeedForward();
    SmartDashboard.putNumber( "Arm Arb FF", m_ArbitraryFeedForward );           
  }

  public void moveArmToPosition( int position )
  {
    m_currentTarget = position;    
    double currentPos = m_armTalon.getSelectedSensorPosition(IntakeArmConstants.kPIDLoopIndex);    
    m_armTalon.set( ControlMode.MotionMagic, currentPos );
  }  

  public boolean isDeployed()
  {
    if( Math.abs( m_armTalon.getSelectedSensorPosition(IntakeArmConstants.kPIDLoopIndex) - IntakeArmConstants.kArmDeployedPosition ) < 50 )
      return true;
    else 
      return false;
  }
  public boolean isHome()
  {
    if( Math.abs( m_armTalon.getSelectedSensorPosition(IntakeArmConstants.kPIDLoopIndex) - IntakeArmConstants.kArmHomePosition ) < 50 )
      return true;
    else 
      return false;
  }

  private double calculateArbitraryFeedForward( )
  {
    double kTicksPerDegree = IntakeArmConstants.kEncoderCountsPerRev * IntakeArmConstants.kArmGearRatio / 360; 
    double currentPos = m_armTalon.getSelectedSensorPosition();    
   
    double degrees = ( currentPos - IntakeArmConstants.kArmDeployedPosition) / kTicksPerDegree;
    double radians = java.lang.Math.toRadians(degrees);
    double cosineScalar = java.lang.Math.cos(radians);
    
    double arbitraryFF = IntakeArmConstants.kArmMaxGravityFF * cosineScalar;
     return arbitraryFF;
  }  
}
