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
import frc.robot.commands.AutoIntakeAlgae;
import frc.robot.commands.AutoMoveElevator;
import frc.robot.commands.AutoRotate;
import frc.robot.commands.AutoShootCoral;
import frc.robot.commands.ClawWheelsStall;
import frc.robot.commands.ClimberSet;
import frc.robot.commands.ElevatorMove;
import frc.robot.commands.IntakeAlgae;
import frc.robot.commands.RotateClaw;
import frc.robot.commands.SetVoltage;
import frc.robot.commands.ShootAlgae;
import frc.robot.commands.ShootCoral;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
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
  private final VisionSubsystem m_visionSubsystem = new VisionSubsystem();
  private final RotateClawSubsystem m_rotateClawSubsystem = new RotateClawSubsystem();
  private final ClimbSubsystem m_climbSubsystem = new ClimbSubsystem();
  private final ClawWheelsSubsystem m_clawWheelsSubsystem = new ClawWheelsSubsystem();
  private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController = new CommandXboxController(
        OperatorConstants.kOperatorControllerPort);
  
  //private final SendableChooser<Command> autoChooser;

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

    // Named commands to be able to be used in the autos
    // NamedCommands.registerCommand(X, newX());

    //NamedCommands.registerCommand("AutoIntakeAlgae", new AutoIntakeAlgae(m_clawWheelsSubsystem));
    //NamedCommands.registerCommand("AutoMoveElevator", new AutoMoveElevator(m_elevatorSubsystem));
    //NamedCommands.registerCommand("AutoRotate", new AutoRotate(m_rotateClawSubsystem, m_clawWheelsSubsystem));
    //NamedCommands.registerCommand("AutoShootCoral", new AutoShootCoral(m_rotateClawSubsystem, m_clawWheelsSubsystem, m_driveSubsystem, m_visionSubsystem, m_elevatorSubsystem, 1));
    

    // Build an auto chooser. (can be changed to specific ones -> https://pathplanner.dev/pplib-build-an-auto.html)
    //autoChooser = AutoBuilder.buildAutoChooser(); // This will use Commands.none() as the default option. Put a value inside brackets for default.

    //SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  public Command getAutonomousCommand() {
    //return autoChooser.getSelected();
    return null;
  }

  // Binds commands to buttons
  private void configureBindings() {
    //m_driverController.a().onTrue(new AlignRobot(m_driveSubsystem, m_visionSubsystem, OperatorConstants.kAprilTagBlue));

    m_operatorController.b().onTrue(new RotateClaw(m_rotateClawSubsystem, RotateClawConstants.kDesiredClawRotationsIntake));
    m_operatorController.rightTrigger().onTrue(new RotateClaw(m_rotateClawSubsystem, RotateClawConstants.kDesiredClawRotationOutake));
    m_operatorController.rightBumper().onTrue(new RotateClaw(m_rotateClawSubsystem, RotateClawConstants.kDesiredClawRotationsIntake));
    
    // can add control to rotate for intake 
    m_operatorController.povUp().onTrue(new AutoMoveElevator(m_elevatorSubsystem, m_rotateClawSubsystem, m_clawWheelsSubsystem, ElevatorConstants.kL4));
    m_operatorController.povDown().onTrue(new ElevatorMove(m_elevatorSubsystem, ElevatorConstants.kBottomPosition));

    m_operatorController.leftTrigger().whileTrue(new SetVoltage(m_elevatorSubsystem));
    // m_operatorController.povLeft().onTrue(new AutoMoveElevator(m_elevatorSubsystem, m_rotateClawSubsystem, m_clawWheelsSubsystem, ElevatorConstants.kL2));

    m_operatorController.povLeft().whileTrue(new ClimberSet(m_climbSubsystem, 0.5));
    m_operatorController.povRight().whileTrue(new ClimberSet(m_climbSubsystem, -0.5)); 
    // // add elevator command when PID
    // m_operatorController.a().whileTrue(new IntakeAlgae(m_clawWheelsSubsystem, ClawWheelsConstants.kIntakeAlgaeSpeed));
    // m_operatorController.y().whileTrue(new ShootAlgae(m_clawWheelsSubsystem, ClawWheelsConstants.kOutakeAlgaeSpeed));
    m_operatorController.x().whileTrue(new ShootCoral(m_clawWheelsSubsystem, ClawWheelsConstants.kOutakeCoralSpeed));
    m_operatorController.y().whileTrue(new ShootCoral(m_clawWheelsSubsystem, -ClawWheelsConstants.kOutakeCoralSpeed));

    m_driverController.x().whileTrue(new RunCommand(() -> m_driveSubsystem.setX(), m_driveSubsystem));
  }

}