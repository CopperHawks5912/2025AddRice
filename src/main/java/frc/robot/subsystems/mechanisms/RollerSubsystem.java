package frc.robot.subsystems.mechanisms;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.DIOConstants;
import frc.robot.Constants.RollerConstants;

import edu.wpi.first.wpilibj.DigitalInput;

public class RollerSubsystem extends SubsystemBase {
  private TalonFX rollerMotor;
  private TalonFXConfiguration rollerMotorConfig;
  private DigitalInput intakeBeamBreakSwitch;
    
  /** Creates a new ElevatorSubsystem. */
  public RollerSubsystem() {
    intakeBeamBreakSwitch = new DigitalInput(DIOConstants.BeamBreakSensorPort);
    rollerMotor= new TalonFX(CANConstants.RollerID);
   
    rollerMotorConfig = new TalonFXConfiguration()
      .withMotorOutput(new MotorOutputConfigs()
        .withNeutralMode( NeutralModeValue.Brake))
      .withSlot0(new Slot0Configs()
        .withKP(1)
        .withKS(0)
        .withKA(0)
        .withKV(0))
      .withCurrentLimits(new CurrentLimitsConfigs()
        .withSupplyCurrentLimit(70)
        .withSupplyCurrentLimitEnable(true));
        rollerMotor.getConfigurator().apply(rollerMotorConfig);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
 
  public boolean isBeamBreakBroke() {
    return !intakeBeamBreakSwitch.get();
  }
  public void intakeCoral(){
    rollerMotor.set(RollerConstants.IntakeCoralSpeed);
  }
  public void outputCoral(){
    rollerMotor.set(RollerConstants.OutputCoralSpeed);
  } 
  public void intakeAlgae(){
    rollerMotor.set(RollerConstants.IntakeAlgaeSpeed);
  }
  public void outputAlgae(){
    rollerMotor.set(RollerConstants.OutputAlgaeSpeed);
  } 
  public void holdAlgae(){
    rollerMotor.set(RollerConstants.HoldAlgaeSpeed);
  } 
  public void stopRoller(){
    rollerMotor.set(0);
  }

}