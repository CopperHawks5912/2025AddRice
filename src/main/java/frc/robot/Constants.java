// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{
  public static final class CANConstants
  {
    public static final int LeftShooter = 10;
    public static final int RightShooter = 11;
    public static final int IntakeGrabberID = 20;
    public static final int IntakeArmID     = 21;
  }

  public static class ControllerConstants
  {
    public static final int kDriverControllerPort = 0;
    public static final int kSecondControllerPort = 1;
    
    public static final int ButtonBlueUpper = 1;
    public static final int ButtonBlueLower =  5;
    public static final int ButtonRedUpper1 =  2;
    public static final int ButtonRedUpper2 =  3;
    public static final int ButtonRedUpper3 =  4;
    public static final int ButtonRedLower1 =  6;
    public static final int ButtonRedLower2 =  7;
    public static final int ButtonRedLower3 =  8;
    public static final int ButtonBlack1    = 10;
    public static final int ButtonBlack2    =  9;
    public static final int HorizontalAxis  = 0;
    public static final int VerticalAxis    = 1;
  }

  public static final class ShooterConstants
  {
    public static final double ShootSpeed = 80.0;
  }
  public static final class IntakeGrabberConstants
  {
    public static final double InputSpeed = 0.6;  
    public static final double OutputSpeed = -0.6;  
  }
  
  public static final class IntakeArmConstants
  {
    public static final int PIDProfileSlotIndex = 0;
    public static final int PIDLoopIndex = 0;

    // public static final double ArmMaxForwardSpeed = 0.05;
    // public static final double ArmMaxReverseSpeed = 0.12;
    
    // public static final double ArmForwardPositionMultiplier = 30;
    // public static final double ArmReversePositionMultiplier = 30;

    public static final double ArmMaxPeakOutputForward = 0.5;//0.7;
    public static final double ArmMaxPeakOutputReverse = -0.5;//-0.7;
    
    public static final double ArmCruiseVelocity = 400;//700;
    public static final double ArmAcceleration = 300;//500;

    public static final int ArmHomePosition = 0;
    public static final int ArmDeployedPosition = 2380; //2440;
    public static final int ArmAmpPosition = 1280; //2440;
    
    public static final double ArmMaxGravityFF = 0.08;  //power required to hold forearm horizontal.
     
    /**
	   * Gains used in Motion Magic, to be adjusted accordingly
     * Gains(kp, ki, kd, kf, izone, peak output);
     */
    public static final Gains ArmGains = new Gains(4, 0.0, 0.0, 0.32, 0, 1.0);
    
    public static final int EncoderCountsPerRev = 4096;
    public static final double ArmGearRatio = 1.0;  //1:1 gearing (100:1 gearbox is in front of the encoder 
  }


  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag

  public static final class AutonConstants
  {

    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
    public static final PIDConstants ANGLE_PID   = new PIDConstants(0.4, 0, 0.01);
  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double LEFT_X_DEADBAND  = 0.1;
    public static final double LEFT_Y_DEADBAND  = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }
}
