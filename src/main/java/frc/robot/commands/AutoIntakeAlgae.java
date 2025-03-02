// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ClawWheelsConstants;
import frc.robot.subsystems.ClawWheelsSubsystem;

public class AutoIntakeAlgae extends SequentialCommandGroup {
  public AutoIntakeAlgae(ClawWheelsSubsystem m_clawWheels) {
    addCommands(
      new IntakeAlgae(m_clawWheels, ClawWheelsConstants.kIntakeAlgaeSpeed)
    );
  }
}