// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ClawWheelsConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.RotateClawConstants;
import frc.robot.subsystems.ClawWheelsSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.RotateClawSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AutoShootCoral extends ParallelCommandGroup {
  public AutoShootCoral(
    RotateClawSubsystem m_rotateClaw, 
    ClawWheelsSubsystem m_clawWheels, 
    DriveSubsystem m_driveSubsystem, 
    VisionSubsystem m_visionSubsystem,
    ElevatorSubsystem m_elevatorSubsystem,
    int m_targetId // Need to figure out how to get this working
  ) {
    addCommands(
      new AlignRobot(m_driveSubsystem, m_visionSubsystem, m_targetId),
      new SequentialCommandGroup (
        new ElevatorMove(m_elevatorSubsystem, m_rotateClaw, ElevatorConstants.kTopPosition),
        new ShootCoral(m_clawWheels, ClawWheelsConstants.kOutakeCoralSpeed).withTimeout(0.5)
      )
    );
  }
}