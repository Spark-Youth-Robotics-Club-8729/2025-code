// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.subsystems.ClawWheelsSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.RotateClawSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveAutoMoveElevator23 extends ParallelRaceGroup {
  /** Creates a new DriveAutoMoveElevator23. */
  public DriveAutoMoveElevator23(DriveSubsystem m_driveSubsystem, ElevatorSubsystem m_elevatorSubsytem, RotateClawSubsystem m_rotateClaw,
      ClawWheelsSubsystem m_clawWheels, double desiredPosition) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new DriveStraight(m_driveSubsystem, 0.15),
      new AutoMoveElevator23(m_elevatorSubsytem, m_rotateClaw, m_clawWheels, desiredPosition)
    );
  }
}
