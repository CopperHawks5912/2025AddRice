package frc.robot.subsystems.mechanisms;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.*;

public class Elevator extends SubsystemBase {
    ElevatorState state = ElevatorState.Home;

    public RelativeEncoder leftEncoder;
    public RelativeEncoder rightEncoder;
    public SparkMax leftMotor;
    public SparkMax rightMotor;

    PIDController backupController;

    SparkClosedLoopController leftController;
    SparkClosedLoopController rightController;

    public Elevator() {            
        leftMotor = new SparkMax(CANConstants.LeftElevatorID, MotorType.kBrushless);
        rightMotor = new SparkMax(CANConstants.LeftElevatorID, MotorType.kBrushless);

        SparkMaxConfig config = new SparkMaxConfig();
        config
            .idleMode(IdleMode.kBrake);
        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(1.5)
            .i(0)
            .d(0)
            .outputRange(-0.3, 0.3)
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(-180, 180);
        config.inverted(false);
        leftMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        config.inverted(true);
        rightMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        leftEncoder = leftMotor.getEncoder(); 
        //leftEncoder.setPosition(0);
        leftController = leftMotor.getClosedLoopController();

        rightEncoder = rightMotor.getEncoder(); 
        //rightEncoder.setPosition(0);
        rightController = rightMotor.getClosedLoopController();
    }

    public Command elevate(ElevatorState floor) {
        return Commands.runOnce(() -> {
            //double rotations = getMotorRotations((floorToMm(floor)-frc.robot.Constants.ElevatorConstants.DOWN));
            double rotations = getStateRotations(floor);
            leftController.setReference(rotations, SparkMax.ControlType.kPosition);
            rightController.setReference(rotations, SparkMax.ControlType.kPosition);
            state = floor;
            SmartDashboard.putString("floor", state.toString()); //reports state
            SmartDashboard.putNumber("rotations", rotations);
        });
    }



    public String getElevatorState() {
        return state.toString();
    }

    public void setHeight() {
        leftController.setReference(getMotorRotations(500), ControlType.kPosition);
    }

    public enum ElevatorState {
        Home,
        Level1,
        Level2,
        Level3,
        Level4,
    }

    private double floorToMm (ElevatorState floor) {//TODO add conversion to DOWN state
        if (floor == ElevatorState.Home) {
            return frc.robot.Constants.ElevatorConstants.Home;
        } if (floor == ElevatorState.Level1) {
            return frc.robot.Constants.ElevatorConstants.Level1;
        } if (floor == ElevatorState.Level2) {
            return frc.robot.Constants.ElevatorConstants.Level2;
        } if (floor == ElevatorState.Level3) {
            return frc.robot.Constants.ElevatorConstants.Level3;
        } if (floor == ElevatorState.Level4) {
            return frc.robot.Constants.ElevatorConstants.Level4;
        } return 0; //invalid floor inputed
    }

    private double getMotorRotations(double height_requested_m_L) { //enter double of elevator extension in mm, returns number of windings of the motor required to achieve that
        double thickness_in_mm_h = 0.0025;
        double inner_diam_in_m_D0 = 0.024;
        double max_height = 1.6;
        double max_windings = 12;
        double requested_height_fraction = height_requested_m_L/max_height;
        if (requested_height_fraction > 1) {
            requested_height_fraction = 1;
        }

        double requested_motor_rotation = Math.abs((thickness_in_mm_h - inner_diam_in_m_D0 + Math.sqrt((Math.pow(inner_diam_in_m_D0 - thickness_in_mm_h, 2) + ((4*thickness_in_mm_h*height_requested_m_L) / (Math.PI))))) / (2*thickness_in_mm_h));

        if (requested_motor_rotation > max_windings) {
            requested_motor_rotation = max_windings;
        }

        return requested_motor_rotation * 5; //5 kvuli prevodovce
    }

    double getStateRotations(ElevatorState state) {
        switch (state) {
            case Home:
                return -15;
            case Level1:
                return 10.0;
            case Level2:
                return 40;
            case Level3:
                return 54;
            default:
                return 0.0;
        }
    }
}