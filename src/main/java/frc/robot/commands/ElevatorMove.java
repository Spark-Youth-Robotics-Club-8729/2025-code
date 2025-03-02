package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.RotateClawConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.RotateClawSubsystem;

public class ElevatorMove extends Command {
    private final ElevatorSubsystem m_elevatorSubsystem;
    private final RotateClawSubsystem m_rotateSubsystem;
    private final double m_targetPosition;

    public ElevatorMove(ElevatorSubsystem subsystem, RotateClawSubsystem claw, double targetPosition) {
        m_elevatorSubsystem = subsystem;
        m_rotateSubsystem = claw;
        m_targetPosition = targetPosition;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        m_elevatorSubsystem.resetPID();
        new SequentialCommandGroup(
            new RotateClaw(m_rotateSubsystem, RotateClawConstants.kDesiredClawRotations),
            this // Refer to current elevator command
        ).schedule();
    }

    // when button pressed rotate
    @Override
    public void execute() {
        m_elevatorSubsystem.rotate(m_elevatorSubsystem.setDesiredPosition(m_targetPosition));
    }

    // when button not pressed stop rotating
    @Override
    public void end(boolean interrupted) {
        m_elevatorSubsystem.holdPosition();
    }

    @Override
    public boolean isFinished() {
        return m_elevatorSubsystem.isAtSetpoint(); // Runs until interrupted
    }
}
