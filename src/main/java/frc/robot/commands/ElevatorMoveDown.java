package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.RotateClawConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.RotateClawSubsystem;

public class ElevatorMoveDown extends Command {
    private final ElevatorSubsystem m_elevatorSubsystem;
    private final double m_targetPosition;

    public ElevatorMoveDown(ElevatorSubsystem subsystem, double targetPosition) {
        m_elevatorSubsystem = subsystem;
        m_targetPosition = targetPosition;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        m_elevatorSubsystem.resetPID();
    }

    @Override
    public void execute() {
        m_elevatorSubsystem.setVoltage(m_elevatorSubsystem.setDesiredPosition(m_targetPosition));
    }
    
    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return (m_elevatorSubsystem.isAtSetpoint(m_elevatorSubsystem.setDesiredPosition(m_targetPosition), m_targetPosition, m_elevatorSubsystem.getPosition()) 
        || 
        m_elevatorSubsystem.checkDownwardsVoltage(m_elevatorSubsystem.setDesiredPosition(m_targetPosition))); 
    }
}