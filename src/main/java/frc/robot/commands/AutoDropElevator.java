package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.RotateClawConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.RotateClawSubsystem;

public class AutoDropElevator extends SequentialCommandGroup {
    public AutoDropElevator(ElevatorSubsystem m_elevatorSubsystem, RotateClawSubsystem m_rotateclaw) {
        addCommands(
            new RotateClaw(m_rotateclaw, RotateClawConstants.kDesiredClawRotationElevator),
            new ElevatorMoveDown(m_elevatorSubsystem, ElevatorConstants.kBottomPosition),
            new SetVoltage(m_elevatorSubsystem, ElevatorConstants.kBottomVoltage),
            new WaitCommand(0.5),
            new ResetEncoders(m_elevatorSubsystem),
            new RotateClaw(m_rotateclaw, RotateClawConstants.kDesiredClawRotationsIntake)
        );
    }
}
