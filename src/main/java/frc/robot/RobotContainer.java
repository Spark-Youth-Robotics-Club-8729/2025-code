// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.commands.AlignRobot;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final VisionSubsystem m_visionSubsystem = new VisionSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    m_driveSubsystem.setDefaultCommand(
                                // The left stick controls translation of the robot.
                                // Turning is controlled by the X axis of the right stick.
                                new RunCommand(
                                                () -> m_driveSubsystem.drive(
                                                                -MathUtil.applyDeadband(
                                                                                m_driverController.getRawAxis(1),
                                                                                OperatorConstants.kDriveDeadband),
                                                                -MathUtil.applyDeadband(
                                                                                m_driverController.getRawAxis(0),
                                                                                OperatorConstants.kDriveDeadband),
                                                                -MathUtil.applyDeadband(
                                                                                m_driverController.getRawAxis(4),
                                                                                OperatorConstants.kDriveDeadband),
                                                                true),
                                                                m_driveSubsystem));

    // m_driveSubsystem.setDefaultCommand(
    //     // Left joystick -> moving 
    //     // Right joystick -> rotation
    //     // Deadband to avoid small movements
    //     new RunCommand(
    //         () -> m_driveSubsystem.drive(
    //             -MathUtil.applyDeadband(m_driverController.getLeftY(), OperatorConstants.kDriveDeadband),
    //             -MathUtil.applyDeadband(m_driverController.getLeftX(), OperatorConstants.kDriveDeadband),
    //             -MathUtil.applyDeadband(m_driverController.getRightX(), OperatorConstants.kDriveDeadband),
    //             true),
    //         m_driveSubsystem));
  }

  // Binds commands to buttons
  private void configureBindings() {
    m_driverController.a().onTrue(new AlignRobot(m_driveSubsystem, m_visionSubsystem, OperatorConstants.kAprilTagBlue));
  }

  public DriveSubsystem getDriveSubsystem() {
    return m_driveSubsystem;
  }

  // Temporary
  public Command getAutonomousCommand() {
    return null; 
  }
}