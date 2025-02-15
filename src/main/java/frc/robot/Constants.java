// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.PIDConstants;

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
  public static class SwerveConstants
  {
    public static final double DriveSpeedPercent = 0.60;//0.99;  //Callum, you're allowed to touch this one, but only this one.
    public static final double RotationSpeedPercent = 0.85;//0.85;  //Ok, maybe this one too.
    
    public static final double MaxSpeed  = Units.feetToMeters(14.5);
    public static final double WheelLockTime = 10; // seconds
    public static final double RobotMass = (100) * 0.453592; // 32lbs * kg per pound
    public static final Matter Chassis    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), RobotMass);
    public static final double LoopTime  = 0.13; //s, 20ms + 110ms sprk max velocity lag 
    
    public static final double Deadband       = 0.1;
    public static final double LeftYDeadbad   = 0.1;
    public static final double RightXDeadband = 0.1;
    public static final double TurnConstant   = 6;
  }

  public static class ControllerConstants
  { 
    public static final int ButtonBlueUpper =  1;
    public static final int ButtonBlueLower =  5;
    public static final int ButtonRedUpper1 =  2;
    public static final int ButtonRedUpper2 =  3;
    public static final int ButtonRedUpper3 =  4;
    public static final int ButtonRedLower1 =  6;
    public static final int ButtonRedLower2 =  7;
    public static final int ButtonRedLower3 =  8;
    public static final int ButtonBlack1    = 10;
    public static final int ButtonBlack2    =  9;
    public static final int HorizontalAxis  =  0;
    public static final int VerticalAxis    =  1;    
  }

  public static final class CANConstants
  {
    public static final int LeftElevatorID   = 11;
    public static final int RightElevatorID  = 12;
  }
  
  public static final class DIOConstants
  {
    // public static final int IntakeArmLimitSwitch = 0;
    public static final int BeamBreakSensorPort = 1;
  }
  
  public static final class AutoConstants
  {
    // public static final double ShooterDelaySeconds = 0.5;
    // public static final double ShooterFastDelaySeconds = 0.20;
    // public static final double IntakeDelaySeconds = 3;
  }

  public static final class ElevatorConstants  //positions in millimeters
  {
    public static double HomePosition = 250.0;
    public static double Lvl1Position = 457.2;
    public static double Lvl2Position = 809.6;
    public static double Lvl3Position = 1209.7;
    public static double Lvl4Position = 1828.8;

    public static double MillimetersToRotations = 10;
    public static double ErrorThreshold = 10.0;
  }

  public static class PWMConstants{
    public static final int LEDStringID = 0;
  }

  public static class LEDConstants{
    public static final int LEDStringLength = 66;
    public static final int LEDModeOff = -1;
    public static final int LEDModeAllianceBlue = 0;
    public static final int LEDModeAllianceRed = 1;  
    public static final int LEDModeRainbow = 2;  
    public static final int LEDModeCopperHawks = 3;  
    public static final int LEDModeNoteEaten = 10;
    public static final int LEDModeShooting = 11;
    
  }

}
