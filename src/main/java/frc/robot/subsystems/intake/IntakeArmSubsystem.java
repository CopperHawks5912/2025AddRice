package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
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
		
    m_armTalon.configPeakOutputForward(IntakeArmConstants.ArmMaxPeakOutputForward, kTimeoutMs);
	  m_armTalon.configPeakOutputReverse(IntakeArmConstants.ArmMaxPeakOutputReverse, kTimeoutMs);
   
    /* Set Motion Magic gains in slot0 - see documentation */
    m_armTalon.selectProfileSlot( IntakeArmConstants.PIDProfileSlotIndex, IntakeArmConstants.PIDLoopIndex);
    m_armTalon.config_kF(IntakeArmConstants.PIDProfileSlotIndex, IntakeArmConstants.ArmGains.kF, kTimeoutMs);
    m_armTalon.config_kP(IntakeArmConstants.PIDProfileSlotIndex, IntakeArmConstants.ArmGains.kP, kTimeoutMs);
    m_armTalon.config_kI(IntakeArmConstants.PIDProfileSlotIndex, IntakeArmConstants.ArmGains.kI, kTimeoutMs);
    m_armTalon.config_kD(IntakeArmConstants.PIDProfileSlotIndex, IntakeArmConstants.ArmGains.kD, kTimeoutMs);    

    /* Set acceleration and vcruise velocity - see documentation */
    m_armTalon.configMotionCruiseVelocity(IntakeArmConstants.ArmCruiseVelocity, kTimeoutMs);
    m_armTalon.configMotionAcceleration( IntakeArmConstants.ArmAcceleration, kTimeoutMs);
    
    /* Zero the sensor once on robot boot up */
    m_armTalon.setSelectedSensorPosition(0, IntakeArmConstants.PIDLoopIndex, kTimeoutMs);	
    m_armTalon.setSensorPhase(false);   
    m_armTalon.setInverted(InvertType.None);  
      
 }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //m_armTalon.setSensorPhase(false);     
 
    // SmartDashboard.putData(this);
    // SmartDashboard.putNumber( "Arm Position", m_armTalon.getSelectedSensorPosition(IntakeArmConstants.PIDLoopIndex) );
    // SmartDashboard.putNumber( "Arm Power", m_armTalon.getMotorOutputPercent() );
    // //SmartDashboard.putNumber( "Arm Target", m_currentTarget.getShoulderPosition() );
    // //SmartDashboard.putBoolean( "Shoulder Switch", m_shoulderLimitSwitch.get() );

    // m_ArbitraryFeedForward = calculateArbitraryFeedForward();
    // SmartDashboard.putNumber( "Arm Arb FF", m_ArbitraryFeedForward );           
  }

  public void moveArmToPosition( int position )
  {
    m_currentTarget = position;    
    //double currentPos = m_armTalon.getSelectedSensorPosition(IntakeArmConstants.PIDLoopIndex);    
    m_armTalon.set( ControlMode.MotionMagic, m_currentTarget, DemandType.ArbitraryFeedForward, m_ArbitraryFeedForward );
  }  

  public boolean isDeployed()
  {
    if( Math.abs( m_armTalon.getSelectedSensorPosition(IntakeArmConstants.PIDLoopIndex) - IntakeArmConstants.ArmDeployedPosition ) < 25 )
      return true;
    else 
      return false;
  }
  public boolean isHome()
  {
    if( Math.abs( m_armTalon.getSelectedSensorPosition(IntakeArmConstants.PIDLoopIndex) - IntakeArmConstants.ArmHomePosition ) < 25 )
      return true;
    else 
      return false;
  }

  private double calculateArbitraryFeedForward( )
  {
    double kTicksPerDegree = IntakeArmConstants.EncoderCountsPerRev * IntakeArmConstants.ArmGearRatio / 360; 
    double currentPos = m_armTalon.getSelectedSensorPosition();    
   
    double degrees = ( currentPos - IntakeArmConstants.ArmDeployedPosition) / kTicksPerDegree;
    double radians = java.lang.Math.toRadians(degrees);
    double cosineScalar = java.lang.Math.cos(radians);
    
    double arbitraryFF = IntakeArmConstants.ArmMaxGravityFF * cosineScalar;
     return arbitraryFF;
  }  
}
