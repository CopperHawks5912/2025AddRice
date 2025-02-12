package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase{
    private TalonFX m_left = new TalonFX(CANConstants.LeftShooterID);
    private TalonFX m_right = new TalonFX(CANConstants.RightShooterID);  
    private TalonFXConfiguration configuration = new TalonFXConfiguration();
    private VelocityVoltage m_velocityVoltage = new VelocityVoltage(0);
    
  public ShooterSubsystem() {    
    configuration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
     m_left.getConfigurator().apply(configuration);
    m_right.getConfigurator().apply(configuration);
    
        var slot0Configs = new Slot0Configs();
        slot0Configs.kV = 0.12;
        slot0Configs.kP = 0.80;
        slot0Configs.kI = 0.1;
        slot0Configs.kD = 0.01;
        m_left.getConfigurator().apply(slot0Configs, 0.050);
        m_velocityVoltage.Slot = 0;

    m_right.setControl(new StrictFollower(m_left.getDeviceID()) );
    //m_right.setInverted(true);  //replaced with "configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;" call above

    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber( "Shooter Speed", m_left.getVelocity().getValueAsDouble() );
    //SmartDashboard.putNumber( "Arm Target", m_currentTarget.getShoulderPosition() );
    //SmartDashboard.putBoolean( "Shoulder Switch", m_shoulderLimitSwitch.get() );
  }
  public void speakerPrelauch() {
       m_left.setControl(m_velocityVoltage.withVelocity(ShooterConstants.PrelaunchTargetSpeed));
      } 
  public void speakerShoot() {
       m_left.setControl(m_velocityVoltage.withVelocity(ShooterConstants.SpeakerShootTargetSpeed));
      } 
  public void ampShoot() {
        m_left.setControl(m_velocityVoltage.withVelocity(ShooterConstants.AmpShootTargetSpeed));
      } 
  public void stopShooter() {
        m_left.setVoltage(0);
      } 
  public boolean isAtSpeakerSpeed() {
    var velocity = m_left.getVelocity();    
    return ( velocity.getValueAsDouble() >= ShooterConstants.SpeakerShootMinSpeed );
  }
  public boolean isAtAmpSpeed() {
    var velocity = m_left.getVelocity();    
    return ( velocity.getValueAsDouble() >= ShooterConstants.AmpShootMinSpeed );
  }
}
