// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ClawWheelsConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.RotateClawConstants;
import frc.robot.subsystems.ClawWheelsSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.RotateClawSubsystem;

public class AutoIntakeCoral extends SequentialCommandGroup {
  public AutoIntakeCoral(ElevatorSubsystem m_elevatorSubsystem, RotateClawSubsystem m_rotateClaw,
      ClawWheelsSubsystem m_clawWheels) {
    addCommands(
        new RotateClaw(m_rotateClaw, RotateClawConstants.kDesiredClawRotationBottom),
        new CoralInIntake(m_clawWheels),
        new ParallelCommandGroup(
            new RotateClaw(m_rotateClaw, RotateClawConstants.kDesiredClawRotationsIntake),
            new ShootCoralVoltage(m_clawWheels, ClawWheelsConstants.kOutakeCoralSpeed).withTimeout(1.55)
        )
    );
  }
}