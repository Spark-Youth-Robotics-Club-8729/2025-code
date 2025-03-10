// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.Matrix;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;


public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final double kDriveDeadband = 0.1;

    public static final int kAprilTagBlue = 21;
    public static final int kAprilTagRed = 10;
  }

  public static class VisionConstants {
    public static final String kCameraName = "testingcam";

    // April Tag Layout for Pose Estimator
    public static final AprilTagFieldLayout TAG_LAYOUT = AprilTagFields.kDefaultField.loadAprilTagLayoutField();

    public static final int kCameraRoll = 0;
    public static final int kCameraPitch = 0;
    public static final double kCameraYaw = Math.PI;


    // Where the camera is mounted with regards to the robot
    public static final Transform3d kRobotToCam = new Transform3d(new Translation3d(0, 0, 0),
      new Rotation3d(kCameraRoll, kCameraPitch * Math.PI/180, kCameraYaw));
        // Translation3d is offset from center or robot/robot origin
        // Rotation3d is rotation on forward/back axis, rotation on left/right axis, rotation on up/down axis

    // Set values for std devs to determine trustworthiness
      // Change with actual values in future
    public static final Matrix<N3, N1> kSingleTagSD = VecBuilder.fill(0, 0, 0); // Should be higher
    public static final Matrix<N3, N1> kMultiTagSD = VecBuilder.fill(0, 0, 0);
  }

  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.1;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(26.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(26.5);
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

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 2;
    public static final int kRearLeftDrivingCanId = 4;
    public static final int kFrontRightDrivingCanId = 8;
    public static final int kRearRightDrivingCanId = 6;

    public static final int kFrontLeftTurningCanId = 1;
    public static final int kRearLeftTurningCanId = 3;
    public static final int kFrontRightTurningCanId = 7;
    public static final int kRearRightTurningCanId = 5;

    public static final boolean kGyroReversed = true;

    //slew
    public static final double kDirectionSlewRate = 2.3; // radians per second default: 1.2
    public static final double kMagnitudeSlewRate = 3.0; // percent per second (1 = 100%) default: 1.8
    public static final double kRotationSlewRate = 2.6; // percent per second (1 = 100%) default: 2.0
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 12;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class RotateClawConstants {
    public static final double kP = 4.2;
    public static final double kI = 3.6;
    public static final double kD = 0;

    public static final int kClawRotationCanId = 11;
    
    public static final double kDesiredClawRotationsIntake = 0.0;
    public static final double kDesiredClawRotationElevator = 0.977;
    public static final double kDesiredClawRotationOutake = 0.94;
    public static final double kDesiredClawRotationOutaking = 0.92;
    // public static final int kClawEncoderDioPort = 0; // Check dio port
  }

  public static final class ElevatorConstants {
    public static final int kRightKrakenCanId = 9;
    public static final int kLeftKrakenCanId = 10;


    public static final int kLimitSwitchPort = 1;

    public static final double kKrakenP = 0.08; // 0.01
    public static final double kKrakenI = 0.008; // 0.0007
    public static final double kKrakenD = 0.0;

    // public static final double kGravityFeedForward = 0.05;
    
    public static final double kKrakenTolerance = 0.3;

    public static final double kL4 = 32.8; //change //32.2 with 0.016
    public static final double kL3 = 20.0;
    public static final double kL2 = 11.0;
    public static final double kL1 = 9.5;
    public static final double kBottomPosition = 0.0; //change

    public static final double k23Algae = 3.0; //change

    // public static final double kLevel4 = 34.0; //change //32.2 with 0.016
    // public static final double kLevel3 = 11.5;
    // public static final double kLevel2 = 0.0; //change
    // public static final double kLevel1 = 0.0; //change
    // public static final double kBottomPosition = 0.0; //change



    public static final double kS = 0.15; //test at min
    public static final double kG = 0.387; //test at max
    public static final double kV = 0.00;
    public static final double kA = 0.00;


    // public static final int kTopLimitSwitchPort = 3;
    // public static final int kBottomLimitSwitchPort = 4;
    
    public static final double kBottomCurrentThreshold = 40.0; // 40A motors
  }

  /** Constants for Claw Wheels Subsystem. */
  public class ClawWheelsConstants {
      public static final double kIntakeAlgaeSpeed = 0.2; // Speed for algae intake
      public static final double kOutakeAlgaeSpeed = 0.5; // Speed for algae outtake
      public static final double kOutakeCoralSpeed = 0.5; // Speed for coral outtake
      public static final double kIntakeAlgaeStall = 0.025;
      public static final int kTopWheelMotorID = 12;

      public static final int kBottomWheelMotorID = 13;

      public static final int kCoralBreakSensorDioPort = 2;
  }

  public static final class ClimbConstants {
    public static final double kP = 1;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kTolerance = 1.0;
    
    public static final int kClimbSpinMotorCanId = 14;
    public static final double kDesiredClimbAngle = 10.0;

    // public static final int kClimbEncoderDioPort = 1; 
  }

}