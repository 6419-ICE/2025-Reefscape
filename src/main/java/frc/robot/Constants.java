// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static class PathPlannerConstants {
    /**get the config from GUI while handling thrown exceptions*/
    private static RobotConfig getConfig() {
      try {
        return RobotConfig.fromGUISettings();
      } catch (IOException | ParseException e) {
        throw new RuntimeException(e);
      }
    }
    public static final RobotConfig config = getConfig();
    public static final PIDConstants translationPID = new PIDConstants(8.25, 0, 0); 
    public static final PIDConstants rotationPID = new PIDConstants(12,0,0); //8.75
  } 

  /**Swerve Module Constants */
  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = false;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.08;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15); //15
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = Math.PI*2; // radians
    public static final double kTurningEncoderVelocityFactor = Math.PI*2 / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = Math.PI*2; // radians

    public static final double kDrivingP = 0.275; //0.055726;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1.0 / 565.0;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 2.125;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -0.75;
    public static final double kTurningMaxOutput = 0.75;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 40; // amps
    public static final int kTurningMotorCurrentLimit = 30; // amps

    public static final double currentDrawAgainstWall = 35;
  }
  public static final class NeoMotorConstants {

    public static final double kFreeSpeedRpm = 5676;
  }
  public static final class MotorIDs {
    /*
     *   public static final int kFrontLeftDrivingCanId = 2;
    public static final int kRearLeftDrivingCanId = 4;
    public static final int kFrontRightDrivingCanId = 8;
    public static final int kRearRightDrivingCanId = 6;

    public static final int kFrontLeftTurningCanId = 3;
    public static final int kRearLeftTurningCanId = 5;
    public static final int kFrontRightTurningCanId = 9;
    public static final int kRearRightTurningCanId = 7;

     */
    public static final int frontLeftDriveID = 2;
    public static final int frontLeftTurnID = 3;

    public static final int frontRightDriveID = 8;
    public static final int frontRightTurnID = 9;

    public static final int backLeftDriveID = 4;
    public static final int backLeftTurnID = 5;

    public static final int backRightDriveID = 6;
    public static final int backRightTurnID = 7;
  }
  public static final class DriveConstants {
    public static final double kMaxSpeedMetersPerSecond = Units.feetToMeters(17.6); 
    public static final double kMaxAngularSpeed = 2*Math.PI; // radians per second //2*math.pi

    // Chassis configuration
    public static final double kTrackWidth = 0.604;
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = 0.604;
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    public static final boolean kGyroReversed = false;
  }
  public static class AprilTagConstants {
    //Two steps for allignment? First gets to correct distance for alignment and heading, then does lateral allignment and drives fwd
    public static final double kReefLateralTolerance = 0.25; //quarter inch tolerance on robot lateral alignment
  }
  public static class OuttakeConstants {

    public static final int outtakeMotorID = 10;
    public static final int rotationMotorID = 11;

    public static final int lineBreakPort = 3;

    public static final double rotationP = 0.15; //0.55
    public static final double rotationI = 0.0;
    public static final double rotationD = 0.0;
    public static final double tolerance = 3;
  }

  public static class IntakeConstants {
    public static final int intakeMotorID1 = 12;

    public static final int lineBreakPort = 0;

  }

  public static class ElevatorConstants {
    public static final int rightMotorID = 13;
    public static final int leftMotorID = 14;

    
    public static final int switchPort = 2;
    public static final double kP = 0.0675; //0.065
    public static final double kI = 0;
    public static final double kD = 0.001;
    public static final double tolerance = 0.25;
  }

  public static class FlipperConstants {
    public static final int rotationMotorID = 15;
    public static final int wheelMotorID = 16;

    public static final double rotationP = 0.0;
    public static final double rotationI = 0.0;
    public static final double rotationD = 0.0;
  }
  public static class HangerConstants {
    public static final int motorID = 17;

    
  }

  private static interface DoubleConstant {double getValue();}

  public static enum OuttakeAngles implements DoubleConstant {
    inside   {public double getValue() {return 0;}},
    intake   {public double getValue() {return -57.5;}}, //-52.5
    lowScore {public double getValue() {return -57.5;}}, //-52.5
    midScore {public double getValue() {return -57.5;}}, //-52.5
    topScore {public double getValue() {return -57.5;}}  //-43.6
  }
  public static enum OuttakeStates implements DoubleConstant {
    intake   {public double getValue() {return 0.25;}},
    outtake  {public double getValue() {return 1;}},
    reverse  {public double getValue() {return -0.25;}},
    idle     {public double getValue() {return 0;}}
  }
  public static enum IntakeStates implements DoubleConstant {
    intake   {public double getValue() {return 0.175;}},
    outtake  {public double getValue() {return -0.5;}},
    idle     {public double getValue() {return 0;}}
  }
  public static enum ElevatorPositions implements DoubleConstant {
    inside {public double getValue() {return 0.0;}},
    intake {public double getValue() {return 0.0;}},
    L1     {public double getValue() {return 0.0;}},
    L2     {public double getValue() {return 10;}},
    L3     {public double getValue() {return 22;}},
    L4     {public double getValue() {return 40.75;}}//38
  }
  public static enum FlipperAngles implements DoubleConstant {
    extended {public double getValue() {return 90;}},
    inside   {public double getValue() {return 0;}}
  }
  public static enum FlipperStates implements DoubleConstant {
    running  {public double getValue() {return 0.0;}},
    idle     {public double getValue() {return 0.0;}}
  }
  public static enum HangerAngles implements DoubleConstant {
    inside {public double getValue() {return 0;}},
    active {public double getValue() {return 270.0;}},
    
  }
  public static enum ElevatorAndOuttakePositions {
    inside(ElevatorPositions.inside,OuttakeAngles.inside),
    intake(ElevatorPositions.intake,OuttakeAngles.intake),
    L1(ElevatorPositions.L1,OuttakeAngles.lowScore),
    L2(ElevatorPositions.L2,OuttakeAngles.midScore),
    L3(ElevatorPositions.L3,OuttakeAngles.midScore),
    L4(ElevatorPositions.L4,OuttakeAngles.topScore)
    ;
    public final OuttakeAngles angle;
    public final ElevatorPositions pos;
    ElevatorAndOuttakePositions(ElevatorPositions pos, OuttakeAngles angle) {
      this.angle = angle;
      this.pos = pos;
    }
    public static ElevatorAndOuttakePositions getLevel(int level) {
      return level == 0 ? intake : valueOf("L" + level);
    }
  }
  public static final class LimelightConstants {
    public static final double kLimelightAngle = 27;
    //height of the center of the limelight lens from the floor
    public static final double kLimelightHeightInches = 5.75;
    public static final PIDController xTranslationController = new PIDController(0.6, 0, 0);
    public static final PIDController zTranslationController = new PIDController(0.6,0,0);
    public static final PIDController rotationController = new PIDController(0.01, 0, 0);
    public static final double reefXDiff = 0.12;
    public static final double reefZDiff = 0.17;
    public static final double reefXTolerance = 0.01;
    public static final double reefHeadingTolerance = 0.05;  
    public static final double reefZTolerance = 0.01;
  }
  public static final class LEDConstants {
    public static final int LEDPort = 0;
    public static final int LEDCount = 5;
  }
}
