package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase{
    private TalonFX m_left = new TalonFX(CANConstants.LeftShooter);
    private TalonFX m_right = new TalonFX(CANConstants.RightShooter);  
    private TalonFXConfiguration configuration = new TalonFXConfiguration();

  public ShooterSubsystem() {    
    configuration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    
     m_left.getConfigurator().apply(configuration);
    m_right.getConfigurator().apply(configuration);
    
    m_right.setControl(new StrictFollower(m_left.getDeviceID()) );
    m_right.setInverted(true);

    
  }
  public void fireShooter() {
        m_left.setVoltage(12);
      } 
  public void stopShooter() {
        m_left.setVoltage(0);
      } 
  public boolean isAtLaunchSpeed() {
    var velocity = m_left.getVelocity();    
    return ( velocity.getValue() >= ShooterConstants.ShootSpeed );
        

  }
}
