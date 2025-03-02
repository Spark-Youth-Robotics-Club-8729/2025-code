// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;

public class AutoMoveElevator extends SequentialCommandGroup {
  public AutoMoveElevator(ElevatorSubsystem m_elevatorSubsystem) {
    addCommands(
      new ElevatorMove(m_elevatorSubsystem, ElevatorConstants.kMidPosition)
    );
  }
}