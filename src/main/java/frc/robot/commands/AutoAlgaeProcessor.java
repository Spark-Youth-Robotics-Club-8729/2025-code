package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ClawWheelsConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.RotateClawConstants;
import frc.robot.subsystems.ClawWheelsSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.RotateClawSubsystem;

public class AutoAlgaeProcessor extends SequentialCommandGroup {
    public AutoAlgaeProcessor(ClawWheelsSubsystem m_clawWheelsSubsystem, RotateClawSubsystem m_rotateclaw) {
        addCommands(
            new RotateClaw(m_rotateclaw, RotateClawConstants.kDesiredClawRotationBottom),
            new ShootAlgae(m_clawWheelsSubsystem, ClawWheelsConstants.kOutakeAlgaeSpeed).withTimeout(0.5)
        );
    }
}
