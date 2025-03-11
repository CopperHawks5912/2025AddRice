// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
    public static final double MaxSpeed  = Units.feetToMeters( 3 ); //14.5);
    public static final double WheelLockTime = 10; // seconds
    public static final double RobotMass = (100) * 0.453592; // 32lbs * kg per pound
    public static final Matter Chassis    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), RobotMass);
    public static final double LoopTime  = 0.13; //s, 20ms + 110ms sprk max velocity lag 
    
    public static final double Deadband       = 0.1;
    public static final double LeftYDeadbad   = 0.1;
    public static final double RightXDeadband = 0.1;
    public static final double TurnConstant   = 6;
  }
  
  public static class ReefPoseConstants
  {
    public static enum ScoringAlignment {
      LEFT,
      CENTER,
      RIGHT
    }

    // setup blue side start scoring poses
    private static Pose2d aprilTag18 = new Pose2d();    
    private static Pose2d blueLeft = aprilTag18.transformBy(Units.inchesToMeters(-3), Units.inchesToMeters(6.47), 180);
    private static Pose2d blueCenter = aprilTag18.transformBy(Units.inchesToMeters(-3), Units.inchesToMeters(0), 180);
    private static Pose2d blueRight = aprilTag18.transformBy(Units.inchesToMeters(-3), Units.inchesToMeters(-6.47), 180);

    // setup red side start scoring poses
    private static Pose2d aprilTag10 = new Pose2d();
    private static Pose2d redLeft = aprilTag10.transformBy(Units.inchesToMeters(-3), Units.inchesToMeters(6.47), 180);
    private static Pose2d redCenter = aprilTag10.transformBy(Units.inchesToMeters(-3), Units.inchesToMeters(0), 180);
    private static Pose2d redRight = aprilTag10.transformBy(Units.inchesToMeters(-3), Units.inchesToMeters(-6.47), 180);

    // mapped reef scoring positions
    private static final HashMap<Integer, Pose2d> leftScoringPoses = getLeftScoringPoses();
    private static final HashMap<Integer, Pose2d> centerScoringPoses = getCenterScoringPoses();
    private static final HashMap<Integer, Pose2d> rightScoringPoses = getRightScoringPoses();
    
    /**
     * Define a hashmap of AprilTag poses that represent our 
     * left side scoring poses for each reef AprilTag
     * @return hashmap 
     */
    private static HashMap<Integer, Pose2d> getLeftScoringPoses() {      
      HashMap<Integer, Pose2d> map = new HashMap<>();

      // left scoring positions surrounding the blue reef
      map.put(18, blueLeft);
      map.put(17, blueLeft.rotateBy(60));
      map.put(22, blueLeft.rotateBy(120));
      map.put(21, blueLeft.rotateBy(180));
      map.put(20, blueLeft.rotateBy(240));
      map.put(19, blueLeft.rotateBy(300));

      // left scoring positions surrounding the red reef
      map.put(10, redLeft);
      map.put(11, redLeft.rotateBy(60));
      map.put(6, redLeft.rotateBy(120));
      map.put(7, redLeft.rotateBy(180));
      map.put(8, redLeft.rotateBy(240));
      map.put(9, redLeft.rotateBy(300));

      // return the map
      return map;
    }
  
    /**
     * Define a hashmap of AprilTag poses that represent
     * our center scoring poses for each reef AprilTag
     * @return hashmap 
     */
    private static HashMap<Integer, Pose2d> getCenterScoringPoses() {
      HashMap<Integer, Pose2d> map = new HashMap<>();

      // center scoring positions surrounding the blue reef
      map.put(18, blueCenter);
      map.put(17, blueCenter.rotateBy(60));
      map.put(22, blueCenter.rotateBy(120));
      map.put(21, blueCenter.rotateBy(180));
      map.put(20, blueCenter.rotateBy(240));
      map.put(19, blueCenter.rotateBy(300));

      // center scoring positions surrounding the red reef
      map.put(10, redCenter);
      map.put(11, redCenter.rotateBy(60));
      map.put(6, redCenter.rotateBy(120));
      map.put(7, redCenter.rotateBy(180));
      map.put(8, redCenter.rotateBy(240));
      map.put(9, redCenter.rotateBy(300));

      // return the map
      return map;
    }
      
    /**
     * Define a hashmap of AprilTag poses that represent our 
     * right side scoring poses for each reef AprilTag
     * @return hashmap 
     */
    private static HashMap<Integer, Pose2d> getRightScoringPoses() {
      HashMap<Integer, Pose2d> map = new HashMap<>();

      // right scoring positions surrounding the blue reef
      map.put(18, blueRight);
      map.put(17, blueRight.rotateBy(60));
      map.put(22, blueRight.rotateBy(120));
      map.put(21, blueRight.rotateBy(180));
      map.put(20, blueRight.rotateBy(240));
      map.put(19, blueRight.rotateBy(300));

      // right scoring positions surrounding the red reef
      map.put(10, redRight);
      map.put(11, redRight.rotateBy(60));
      map.put(6, redRight.rotateBy(120));
      map.put(7, redRight.rotateBy(180));
      map.put(8, redRight.rotateBy(240));
      map.put(9, redRight.rotateBy(300));

      // return the map
      return map;
    }
  
    /**
     * Get a left/center/right scoring position based off of a given AprilTag ID
     * @param tagId The fiducial ID of the reef AprilTag to align with
     * @param align Which scoring alignment to move to relative to the center of the AprilTag
     * @return Pose2d
     */
    public static Pose2d getScoringPose(int tagId, ScoringAlignment align) {
      // get the left scoring pose
      if (align == ScoringAlignment.LEFT) {
        return leftScoringPoses.get(tagId);
      }

      // get the right scoring pose
      if (align == ScoringAlignment.RIGHT) {
        return rightScoringPoses.get(tagId);
      }

      // default -> get the center scoring pose
      return centerScoringPoses.get(tagId);
    }

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

    public static final int ArmID  = 13;
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
    public static double HomePosition = 0.0;
    public static double Lvl1Position = 457.2;
    public static double Lvl2Position = 809.6;
    public static double Lvl3Position = 1209.7;
    public static double Lvl4Position = 1828.8;

    public static double MillimetersToRotations = 10;
    public static double ErrorThreshold = 10.0;
  }

  public static final class ArmConstants
  {
    public static double ErrorThreshold = 10.0; 
    
    public static double HomePosition = 0.0;
    public static double Lvl1Position = 457.2;
    public static double Lvl2Position = 809.6;
    public static double Lvl3Position = 1209.7;
    public static double Lvl4Position = 1828.8;
    public static double FloorAlgaePosition = 1828.8;
    public static double ScoreAlgaePosition = 1828.8;
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
