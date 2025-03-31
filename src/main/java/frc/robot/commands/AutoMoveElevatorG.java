// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ClawWheelsConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.RotateClawConstants;
import frc.robot.subsystems.ClawWheelsSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.RotateClawSubsystem;

public class AutoMoveElevatorG extends SequentialCommandGroup {
  public AutoMoveElevatorG(ElevatorSubsystem m_elevatorSubsystem, RotateClawSubsystem m_rotateClaw,
      ClawWheelsSubsystem m_clawWheels, double desiredPosition) {
    addCommands(
        // new ShootCoral(m_clawWheels, 0.4).withTimeout(0.2),
        new RotateClaw(m_rotateClaw, RotateClawConstants.kDesiredClawRotationElevator),
        new ElevatorMove(m_elevatorSubsystem, desiredPosition),
        new SetVoltageG(m_elevatorSubsystem, 0.39075)
    );
  }
}