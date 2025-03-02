// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.RotateClawConstants;
import frc.robot.subsystems.ClawWheelsSubsystem;
import frc.robot.subsystems.RotateClawSubsystem;

public class AutoRotate extends SequentialCommandGroup {
  public AutoRotate(RotateClawSubsystem m_rotateClaw, ClawWheelsSubsystem m_clawWheels) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new RotateClaw(m_rotateClaw, RotateClawConstants.kDesiredClawRotations),
      new WaitCommand(500),
      new ShootCoral(m_clawWheels, 0.1).withTimeout(0.2)
    );
  }
}