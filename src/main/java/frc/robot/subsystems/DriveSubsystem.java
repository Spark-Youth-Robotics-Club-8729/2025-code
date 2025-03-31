// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;

import org.photonvision.PhotonCamera;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.studica.frc.AHRS;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.utils.SwerveUtils;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  private final Field2d m_field = new Field2d();

  // The gyro sensor
  public final AHRS m_gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);

  private VisionSubsystem m_visionSubsystem = new VisionSubsystem();

  // Slew rate filter variables for controlling lateral acceleration
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  private final SlewRateLimiter m_magnitudeLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private final SlewRateLimiter m_directionLimiter = new SlewRateLimiter(DriveConstants.kDirectionSlewRate);
  private final SlewRateLimiter m_rotationLimiter = new SlewRateLimiter(DriveConstants.kRotationSlewRate);

  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

  public PIDController xController = new PIDController(0.1, 0.0, 0.0);
  public PIDController yController = new PIDController(0.1, 0.0, 0.0);
  public PIDController thetaController = new PIDController(0.1, 0.0, 0.0);

  // Odometry class for tracking robot pose
  SwerveDrivePoseEstimator m_odometry = new SwerveDrivePoseEstimator(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(-m_gyro.getYaw()),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      }, new Pose2d());

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // Usage reporting for MAXSwerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);

    RobotConfig config = null;

    try{
      config = RobotConfig.fromGUISettings(); // Config inside the pathplanner gui
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();

      // Add backup config once know actual values:
      /*
      config = new RobotConfig(
          DriveConstants.kRobotMass, //MassKG
          5.0, //MOI
          new ModuleConfig(5.0, 0.0, 0.0, 5.0, 0.0, 0.0, 3.0, 12.0), //Default ModuleConfig
          DriveConstants.kTrackWidth //trackwidthMeters
        );
      */

    }
    // Configure AutoBuilder last
    AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(1.0, 0.0, 0.0), // Translation PID constants (different from module PID, makes sure it follows trajectory)
                    new PIDConstants(1.0, 0.0, 0.0) // Rotation PID constants
            ),
            config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red; // Will mirror if on red side
              }
              return false;
            },
            this // Reference to this subsystem to set requirements (mandatory for autos)
    );
    SmartDashboard.putData("Field", m_field);
  }

  
  // Gets the robot's current relative speeds
  public ChassisSpeeds getRobotRelativeSpeeds() {
    // change
    return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
  }

  private void driveRobotRelative(ChassisSpeeds speeds) {
    drive(speeds, false);
  }

  private void drive(ChassisSpeeds speeds, boolean fieldRelative) {
    if (fieldRelative)
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getPose().getRotation());
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    setModuleStates(swerveModuleStates);
  } 

  private SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState()
    };
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        Rotation2d.fromDegrees(-m_gyro.getYaw()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
            
        });

        // if (DriverStation.isTeleopEnabled()) {
          var visionEst = m_visionSubsystem.getEstimatedGlobalPose();

          visionEst.ifPresent(

                est -> {

                    // Change our trust in the measurement based on the tags we can see

                    // var estStdDevs = m_visionSubsystem.getEstimationStdDevs(m_odometry.getEstimatedPosition());


                    m_odometry.addVisionMeasurement(

                            est.estimatedPose.toPose2d(), est.timestampSeconds); //estStdDevs

                });
        // }

    //SmartDashboard.putNumber("Yaw", -m_gyro.getYaw());
    SmartDashboard.putNumber("ROBOT X", getPose().getX());
    SmartDashboard.putNumber("ROBOT Y", getPose().getY());
    SmartDashboard.putNumber("ROBOT Z", getPose().getRotation().getDegrees());

    double angle = getPose().getRotation().getDegrees();
    if (angle > 0.0) {
      angle -= 180.0;
    } else {
      angle += 180.0;
    }

    SmartDashboard.putNumber("ROBOT Z changed", angle);
    m_field.setRobotPose(getPose());
  }
  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getEstimatedPosition();
  }

  public Pose2d getPoseZero() {
    return new Pose2d(0.0, 0.0, new Rotation2d(0.0));
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(-m_gyro.getYaw()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    double inputTranslationMag = Math.hypot(xSpeed, ySpeed);
    double inputTranslationDir = Math.atan2(ySpeed, xSpeed);

    // Calculate the direction slew rate dynamically
    double directionSlewRate = (m_currentTranslationMag > 1e-4) 
        ? Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag) // Normal speed slew rate
        : 500.0;  // Slew rate for low speeds (instant)

    double currentTime = WPIUtilJNI.now() * 1e-6;
    double elapsedTime = currentTime - m_prevTime;
    double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);

    if (angleDif < 0.4 * Math.PI) {  
        // Small angle -> smoothly adjust direction and magnitude
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magnitudeLimiter.calculate(inputTranslationMag);
    } else if (angleDif > 0.9 * Math.PI) {  
        // Large angle -> slow down briefly then reverse
        m_currentTranslationMag *= 0.4;  // Reduce speed 
        if (m_currentTranslationMag < 0.1) {  // Flip once speed is low enough
            m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
        }
    } else {  
        // Mid-range angle change → gradual transition
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag *= 0.6;  // Reduce speed instead of fully stopping
    }

    m_prevTime = currentTime;

    // Convert to cartesian
    double xSpeedLimited = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
    double ySpeedLimited = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
    double rotLimited = m_rotationLimiter.calculate(rot);

    SmartDashboard.putNumber("Rot", rot);
    SmartDashboard.putNumber("Rot limited", rotLimited);

    // Scale with max speed
    xSpeedLimited *= DriveConstants.kMaxSpeedMetersPerSecond;
    ySpeedLimited *= DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rotLimited * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedLimited, ySpeedLimited, rotDelivered,
                Rotation2d.fromDegrees(-m_gyro.getYaw()))
            : new ChassisSpeeds(xSpeedLimited, ySpeedLimited, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }


  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  public void stop() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
  }


  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(-m_gyro.getYaw()).getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public Pose2d chooseAprilTag() {
    Pose2d robotPose = getPose();
    Pose2d desiredPose = new Pose2d();

    boolean good = false;

    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red && robotPose.getX()>8.775) {
      Map<String, Double> distances = new HashMap<>();
    
      distances.put("6", Math.sqrt(Math.pow(robotPose.getX()-13.47, 2) + Math.pow(robotPose.getY()-3.31, 2)));
      distances.put("7", Math.sqrt(Math.pow(robotPose.getX()-13.89, 2) + Math.pow(robotPose.getY()-4.03, 2)));
      distances.put("8", Math.sqrt(Math.pow(robotPose.getX()-13.47, 2) + Math.pow(robotPose.getY()-4.75, 2)));
      distances.put("9", Math.sqrt(Math.pow(robotPose.getX()-12.64, 2) + Math.pow(robotPose.getY()-4.75, 2)));
      distances.put("10", Math.sqrt(Math.pow(robotPose.getX()-12.23, 2) + Math.pow(robotPose.getY()-4.03, 2)));
      distances.put("11", Math.sqrt(Math.pow(robotPose.getX()-12.64, 2) + Math.pow(robotPose.getY()-3.31, 2)));

      String minKey = Collections.min(distances.entrySet(), Map.Entry.comparingByValue()).getKey();
      double minValue = distances.get(minKey);
      int aprilTag = Integer.parseInt(minKey);

      desiredPose = m_visionSubsystem.returnAprilTagPose(aprilTag);
      good = true;

    } else if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue && robotPose.getX()<8.775) {
      Map<String, Double> distances = new HashMap<>();
    
      distances.put("17", Math.sqrt(Math.pow(robotPose.getX()-4.07, 2) + Math.pow(robotPose.getY()-3.31, 2)));
      distances.put("18", Math.sqrt(Math.pow(robotPose.getX()-3.66, 2) + Math.pow(robotPose.getY()-4.03, 2)));
      distances.put("19", Math.sqrt(Math.pow(robotPose.getX()-4.07, 2) + Math.pow(robotPose.getY()-4.75, 2)));
      distances.put("20", Math.sqrt(Math.pow(robotPose.getX()-4.90, 2) + Math.pow(robotPose.getY()-4.75, 2)));
      distances.put("21", Math.sqrt(Math.pow(robotPose.getX()-5.32, 2) + Math.pow(robotPose.getY()-4.03, 2)));
      distances.put("22", Math.sqrt(Math.pow(robotPose.getX()-4.90, 2) + Math.pow(robotPose.getY()-3.31, 2)));

      String minKey = Collections.min(distances.entrySet(), Map.Entry.comparingByValue()).getKey();
      double minValue = distances.get(minKey);
      int aprilTag = Integer.parseInt(minKey);

      desiredPose = m_visionSubsystem.returnAprilTagPose(aprilTag);
      good = true;
    }

    if (good) {
      return desiredPose;
    }
    return null;

  }
  

  public void pathToPose() {
    Pose2d target_pose;
    target_pose = chooseAprilTag();

    if (target_pose == null) {
      return;
    }

    // target_pose = new Pose2d(0, 0, new Rotation2d(0));

    // double angle = target_pose.getRotation().getDegrees();
    // if (angle > 0.0) {
    //   angle -= 180.0;
    // } else {
    //   angle += 180.0;
    // }

    // List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(new Pose2d(getPose().getX(), getPose().getY(), new Rotation2d(angle*Math.PI/180.0)), target_pose);
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(new Pose2d(getPose().getX(), getPose().getY(), getPose().getRotation()), target_pose);

    PathConstraints constraints = new PathConstraints(0.5, 0.5, Math.PI, 2*Math.PI);

    PathPlannerPath path = new PathPlannerPath(waypoints, constraints, null, new GoalEndState(0.0, Rotation2d.fromDegrees(target_pose.getRotation().getDegrees())));
    path.preventFlipping = true;

    AutoBuilder.followPath(path).schedule();
  }

}