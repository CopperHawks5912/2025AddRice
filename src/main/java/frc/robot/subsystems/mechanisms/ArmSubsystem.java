package frc.robot.subsystems.mechanisms;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class ArmSubsystem extends SubsystemBase {
  private TalonFX armMotor;
  private TalonFXConfiguration armMotorConfig;
  private double targetPos;
  private double zeroPoint;

  public ArmSubsystem() {
    armMotor = new TalonFX(CANConstants.ArmID);
    
    armMotorConfig = new TalonFXConfiguration()
      .withMotorOutput(new MotorOutputConfigs()
        .withNeutralMode( NeutralModeValue.Brake))
      .withSlot0(new Slot0Configs()
        .withKP(1)
        .withKS(0)
        .withKA(0)
        .withKV(0))
        .withCurrentLimits(new CurrentLimitsConfigs()
        .withSupplyCurrentLimit(30)
        .withSupplyCurrentLimitEnable(true));
    armMotor.getConfigurator().apply(armMotorConfig);  
    
    // Initialize dashboard values
    SmartDashboard.setDefaultNumber("Target Position", 0);
    SmartDashboard.setDefaultNumber("Target Velocity", 0);
    SmartDashboard.setDefaultBoolean("Control Mode", false);
    SmartDashboard.setDefaultBoolean("Reset Encoder", false);

    zeroPoint = armMotor.getPosition().getValueAsDouble();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm Pos", armMotor.getPosition().getValueAsDouble() );
    SmartDashboard.putNumber("Arm Target",targetPos );
    isArmAtPose();
  }
  public void setArmPosition(double position){
    targetPos = position + zeroPoint;  
    PositionVoltage voltReq = new PositionVoltage(0);
    voltReq.Position = targetPos;
    voltReq.Velocity = 0.1;    
    armMotor.setControl(voltReq);
    SmartDashboard.putNumber("Target Position", position);    
  }
  public boolean isArmAtPose() {
    boolean atPose = Math.abs( armMotor.getPosition().getValueAsDouble() - targetPos ) < ElevatorConstants.ErrorThreshold; 
    SmartDashboard.putNumber("Arm Error",  armMotor.getClosedLoopError().getValueAsDouble() );
    SmartDashboard.putBoolean("Arm At Pose", atPose );
    return atPose;
  }

   public boolean isArmInAlgaeMode() {
     if( targetPos == ArmConstants.ProcessorAlgaePosition ||
         targetPos == ArmConstants.LowerAlgaePosition ||
         targetPos == ArmConstants.UpperAlgaePosition ||
         targetPos == ArmConstants.NetAlgaePosition )
        return true;
      else 
        return false;
   }
}
