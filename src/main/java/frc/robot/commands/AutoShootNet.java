// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ClawWheelsConstants;
import frc.robot.Constants.RotateClawConstants;
import frc.robot.subsystems.ClawWheelsSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.RotateClawSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoShootNet extends SequentialCommandGroup {
  /** Creates a new AutoShootNet. */
  public AutoShootNet(ElevatorSubsystem m_elevatorSubsystem, RotateClawSubsystem m_rotateClaw,
      ClawWheelsSubsystem m_clawWheels, double desiredPosition) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new RotateClaw(m_rotateClaw, RotateClawConstants.kDesiredClawRotationElevator),
      new ElevatorMove(m_elevatorSubsystem, desiredPosition),
      new RotateClaw(m_rotateClaw, RotateClawConstants.kDesiredClawRotationOutaking),
      new ParallelCommandGroup(new ShootAlgae(m_clawWheels, ClawWheelsConstants.kShootNetAlgaeSpeed).withTimeout(0.9),
            new RotateClaw(m_rotateClaw, RotateClawConstants.kDesiredClawRotationBottom))
    );
  }
}
