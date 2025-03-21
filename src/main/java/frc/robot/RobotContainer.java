// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.*;


import frc.robot.subsystems.ClawWheelsSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.RotateClawSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.commands.AlignRobot;
import frc.robot.commands.AutoAlgaeProcessor;
import frc.robot.commands.AutoDropElevator;
import frc.robot.commands.AutoIntakeCoral;
import frc.robot.commands.AutoMoveElevator;
import frc.robot.commands.AutoMoveElevator1;
import frc.robot.commands.AutoMoveElevator23;
import frc.robot.commands.AutoMoveElevatorAlgae;
import frc.robot.commands.AutoRotate;
import frc.robot.commands.AutoShootCoral;
import frc.robot.commands.AutoShootNet;
import frc.robot.commands.ClawWheelsStall;
import frc.robot.commands.ClimberSet;
import frc.robot.commands.DriveAutoMoveElevator23;
import frc.robot.commands.DriveStraight;
import frc.robot.commands.ElevatorMove;
import frc.robot.commands.IntakeAlgae;
import frc.robot.commands.MoveToPositionCommand;
import frc.robot.commands.MovingToAprilTag;
import frc.robot.commands.RotateClaw;
import frc.robot.commands.SetVoltage;
import frc.robot.commands.ShootAlgae;
import frc.robot.commands.ShootCoral;
import frc.robot.commands.ShootCoralVoltage;
import frc.robot.commands.TranslateRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  // private final VisionSubsystem m_visionSubsystem = new VisionSubsystem();
  private final RotateClawSubsystem m_rotateClawSubsystem = new RotateClawSubsystem();
  private final ClimbSubsystem m_climbSubsystem = new ClimbSubsystem();
  private final ClawWheelsSubsystem m_clawWheelsSubsystem = new ClawWheelsSubsystem();
  private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
  public static final DriveSubsystem m_driveSubsystem = new DriveSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController = new CommandXboxController(
        OperatorConstants.kOperatorControllerPort);
  
  private final SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    //m_clawWheelsSubsystem.setDefaultCommand(new ClawWheelsStall(m_clawWheelsSubsystem, ClawWheelsConstants.kIntakeAlgaeStall));
    
    m_driveSubsystem.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_driveSubsystem.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OperatorConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OperatorConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OperatorConstants.kDriveDeadband),
                true),
                m_driveSubsystem));

    // Named commands to be able to be use

    // NamedCommands.registerCommand(X, newX());

    // NamedCommands.registerCommand("AutoIntakeAlgae", new AutoIntakeAlgae(m_clawWheelsSubsystem));
    NamedCommands.registerCommand("AutoIntakeAlgae23", new AutoMoveElevatorAlgae(m_elevatorSubsystem, m_rotateClawSubsystem, m_clawWheelsSubsystem, ElevatorConstants.k23Algae));
    // NamedCommands.registerCommand("AutoReleaseAlgae", new ShootAlgae(m_clawWheelsSubsystem, ClawWheelsConstants.kOutakeAlgaeSpeed).withTimeout(0.5));
    NamedCommands.registerCommand("AutoMoveElevatorL4", new AutoMoveElevator(m_elevatorSubsystem, m_rotateClawSubsystem, m_clawWheelsSubsystem, ElevatorConstants.kL4));
    NamedCommands.registerCommand("AutoMoveElevatorL3", new AutoMoveElevator23(m_elevatorSubsystem, m_rotateClawSubsystem, m_clawWheelsSubsystem, ElevatorConstants.kL3));
    NamedCommands.registerCommand("AutoMoveElevatorL2", new AutoMoveElevator23(m_elevatorSubsystem, m_rotateClawSubsystem, m_clawWheelsSubsystem, ElevatorConstants.kL2));
    NamedCommands.registerCommand("AutoMoveElevatorL1", new AutoMoveElevator1(m_elevatorSubsystem, m_rotateClawSubsystem, m_clawWheelsSubsystem, ElevatorConstants.kL1));
    NamedCommands.registerCommand("DropElevator", new AutoDropElevator(m_elevatorSubsystem, m_rotateClawSubsystem));
    NamedCommands.registerCommand("AutoRotate", new AutoRotate(m_rotateClawSubsystem, m_clawWheelsSubsystem));
    // NamedCommands.registerCommand("AutoShootCoral", new AutoShootCoral(m_rotateClawSubsystem, m_clawWheelsSubsystem, m_driveSubsystem, m_visionSubsystem, m_elevatorSubsystem, 1));
    // NamedCommands.registerCommand("RightL4", new AlignRobot(m_driveSubsystem, m_visionSubsystem, true));
    // NamedCommands.registerCommand("LeftL4", new AlignRobot(m_driveSubsystem, m_visionSubsystem, false));
    NamedCommands.registerCommand("IntakeCoral", new AutoIntakeCoral(m_elevatorSubsystem, m_rotateClawSubsystem, m_clawWheelsSubsystem));
    NamedCommands.registerCommand("AlgaeNet", new AutoShootNet(m_elevatorSubsystem, m_rotateClawSubsystem, m_clawWheelsSubsystem, ElevatorConstants.kL4));

    // Build an auto chooser. (can be changed to specific ones -> https://pathplanner.dev/pplib-build-an-auto.html)
    autoChooser = AutoBuilder.buildAutoChooser(); // This will use Commands.none() as the default option. Put a value inside brackets for default.

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  // Binds commands to buttons
  private void configureBindings() {
    // m_driverController.a().whileTrue(new AlignRobot(m_driveSubsystem, m_visionSubsystem));

    // m_operatorController.b().onTrue(new RotateClaw(m_rotateClawSubsystem, RotateClawConstants.kDesiredClawRotationsIntake));
    //m_operatorController.rightTrigger().onTrue(new RotateClaw(m_rotateClawSubsystem, RotateClawConstants.kDesiredClawRotationOutake));
    //m_operatorController.rightBumper().onTrue(new RotateClaw(m_rotateClawSubsystem, RotateClawConstants.kDesiredClawRotationsIntake));
    
    // can add control to rotate for intake 
    m_operatorController.y().onTrue(new AutoMoveElevator(m_elevatorSubsystem, m_rotateClawSubsystem, m_clawWheelsSubsystem, ElevatorConstants.kL4));
    m_operatorController.x().onTrue(new AutoMoveElevator23(m_elevatorSubsystem, m_rotateClawSubsystem, m_clawWheelsSubsystem, ElevatorConstants.kL3));
    m_operatorController.b().onTrue(new AutoMoveElevator23(m_elevatorSubsystem, m_rotateClawSubsystem, m_clawWheelsSubsystem, ElevatorConstants.kL2));
    m_operatorController.a().onTrue(new AutoMoveElevator1(m_elevatorSubsystem, m_rotateClawSubsystem, m_clawWheelsSubsystem, ElevatorConstants.kL1));
    m_operatorController.povDown().onTrue(new AutoDropElevator(m_elevatorSubsystem, m_rotateClawSubsystem));

    m_operatorController.leftBumper().onTrue(new AutoMoveElevatorAlgae(m_elevatorSubsystem, m_rotateClawSubsystem, m_clawWheelsSubsystem, ElevatorConstants.k23Algae));
    m_operatorController.rightBumper().onTrue(new AutoMoveElevatorAlgae(m_elevatorSubsystem, m_rotateClawSubsystem, m_clawWheelsSubsystem, ElevatorConstants.k34Algae));
    m_operatorController.leftTrigger().onTrue(new AutoAlgaeProcessor(m_clawWheelsSubsystem, m_rotateClawSubsystem));
    m_operatorController.povRight().whileTrue(new ShootCoral(m_clawWheelsSubsystem, 0.15));
    m_operatorController.rightTrigger().whileTrue(new ShootCoral(m_clawWheelsSubsystem, -0.15));
    m_operatorController.povLeft().onTrue(new AutoIntakeCoral(m_elevatorSubsystem, m_rotateClawSubsystem, m_clawWheelsSubsystem));
    m_operatorController.povUp().onTrue(new AutoShootNet(m_elevatorSubsystem, m_rotateClawSubsystem, m_clawWheelsSubsystem, ElevatorConstants.kL4));

    // // add elevator command when PID
    //m_operatorController.a().whileTrue(new IntakeAlgae(m_clawWheelsSubsystem, ClawWheelsConstants.kIntakeAlgaeSpeed));
    //m_operatorController.y().whileTrue(new ShootCoral(m_clawWheelsSubsystem, -ClawWheelsConstants.kOutakeCoralSpeed));
    // m_driverController.b().whileTrue(new DriveStraight(m_driveSubsystem, 0.15));
    // m_driverController.x().whileTrue(new TranslateRobot(m_driveSubsystem, m_visionSubsystem));
    // m_driverController.x().onTrue(new TestTrajectory(m_driveSubsystem, m_visionSubsystem));
    // m_driverController.povUp().onTrue(new DriveAutoMoveElevator23(m_driveSubsystem, m_elevatorSubsystem, m_rotateClawSubsystem, m_clawWheelsSubsystem, ElevatorConstants.kL2));
    // m_driverController.povRight().onTrue(new DriveAutoMoveElevator23(m_driveSubsystem, m_elevatorSubsystem, m_rotateClawSubsystem, m_clawWheelsSubsystem, ElevatorConstants.kL3));
    // m_driverController.x().onTrue(new MoveToPositionCommand(m_driveSubsystem, m_visionSubsystem.getTagPoseTrajectory(VisionConstants.kAprilTagIds, 0)));
    m_driverController.leftTrigger().whileTrue(new RunCommand(() -> m_driveSubsystem.setX(), m_driveSubsystem));
    // m_driverController.y().whileTrue(new MoveToPositionCommand(m_driveSubsystem, m_visionSubsystem.getTagPoseTrajectory(VisionConstants.kAprilTagIds, 0.0)));
    m_driverController.povUp().whileTrue(new RunCommand(() -> m_driveSubsystem.m_gyro.zeroYaw(), m_driveSubsystem));
    m_driverController.x().onTrue(new RunCommand(() -> m_driveSubsystem.pathToPose(), m_driveSubsystem));
  }

  
}