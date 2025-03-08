package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorMove extends Command {
    private final ElevatorSubsystem m_elevatorSubsystem;
    private final double m_targetPosition;

    public ElevatorMove(ElevatorSubsystem subsystem, double targetPosition) {
        m_elevatorSubsystem = subsystem;
        m_targetPosition = targetPosition;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        m_elevatorSubsystem.resetPID();
    }

    // when button pressed rotate
    @Override
    public void execute() {
        m_elevatorSubsystem.setVoltage(m_elevatorSubsystem.setDesiredPosition(m_targetPosition));
    }

    // when button not pressed stop rotating
    //@Override
    //public void end(boolean interrupted) {
    //    m_elevatorSubsystem.stop();
    //}
    
    // when button not pressed stop rotating
    @Override
    public void end(boolean interrupted) {
        m_elevatorSubsystem.holdPosition();
    }

    @Override
    public boolean isFinished() {
        return m_elevatorSubsystem.isAtSetpoint(m_elevatorSubsystem.setDesiredPosition(m_targetPosition), m_targetPosition, m_elevatorSubsystem.getPosition()); // Runs until interrupted
    }
}