package frc.robot.subsystems.mechanisms;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;
import edu.wpi.first.wpilibj.Preferences;

public class ElevatorSubsystem extends SubsystemBase {
  private TalonFX leftElevatorMotor;
  private TalonFX rightElevatorMotor;
  private TalonFXConfiguration eleMotorConfig;
  private double zeroPoint;

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    leftElevatorMotor= new TalonFX(CANConstants.LeftElevatorID);
    rightElevatorMotor = new TalonFX(CANConstants.RightElevatorID);
    
    eleMotorConfig = new TalonFXConfiguration()
      .withSlot0(new Slot0Configs()
        .withKP(1)
        .withKS(0)
        .withKA(0)
        .withKV(0))
      .withCurrentLimits(new CurrentLimitsConfigs()
        .withSupplyCurrentLimit(100)
        .withSupplyCurrentLimitEnable(true));
      //We are not yet sure on whether or not we are using MotionMagic.
        //.withMotionMagic(new MotionMagicConfigs()
        //.withMotionMagicAcceleration(2491)
        //.withMotionMagicCruiseVelocity(2491)
        //.withMotionMagicJerk(2491));
    leftElevatorMotor.getConfigurator().apply(eleMotorConfig);
    rightElevatorMotor.getConfigurator().apply(eleMotorConfig);
    
    rightElevatorMotor.setControl(new Follower(CANConstants.LeftElevatorID, true));
  }
  private void logMotors(){
//     motorLogger1.log(elevatorMotor1);
//     motorLogger2.log(elevatorMotor2);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(Preferences.getBoolean("Motor Logging", false)){
    logMotors();
    }
    if(leftElevatorMotor.getForwardLimit().getValueAsDouble() > 0.1){
      setZero(ElevatorConstants.HomePosition);
    }
  }
  /**
   * Creates a zero from input
   * @param theDistance the distance that the distance sensor at the bottom of the elevator reads
   */
  public void setZero(double theDistance){//Replace with sensor return
    double rof0 = theDistance * ElevatorConstants.MillimetersToRotations;
    zeroPoint = leftElevatorMotor.getPosition().getValueAsDouble() - rof0;    
    }
  /**
   * Sets the elevator to a position relative to the 0 set by createZero. 
   * @param height double that controls how many millimeters from the distance sensor
   */
  public void setElevatorPosition(double height){
    double position = height * ElevatorConstants.MillimetersToRotations;
    double uPos = position + zeroPoint;
    PositionVoltage voltReq = new PositionVoltage(0);
    leftElevatorMotor.setControl(voltReq.withPosition(uPos));
  }
  public boolean isElevatorAtPose() {
    return leftElevatorMotor.getClosedLoopError().getValueAsDouble() < ElevatorConstants.ErrorThreshold;
  }
  public double getPIDTarget() {
    return leftElevatorMotor.getClosedLoopReference().getValueAsDouble();
  }
  public void stopElevator(){
    leftElevatorMotor.set(0);
  }

}