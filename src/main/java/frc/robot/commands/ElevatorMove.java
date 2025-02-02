package frc.robot.commands;

import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorMove extends Command {
    private final ElevatorSubsystem m_elevatorSubsystem;
    private final PIDController m_pidController;
    private final double m_targetPosition;

    public ElevatorMove(ElevatorSubsystem subsystem, double speed) {
        m_elevatorSubsystem = subsystem;
        m_targetPosition = targetPosition;
        m_pidController = new PIDController(ElevatorConstants.kKrakenP, ElevatorConstants.kKrakenI, ElevatorConstants.kKrakenD);
        m_pidController.setTolerance(0.8);
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        m_pidController.reset()
    }

    // when button pressed rotate
    @Override
    public void execute() {
        double output = m_pidController.calculate(m_elevatorSubsystem.getPosition(), m_targetPosition);
        output = Math.max(-1.0, Math.min(1.0, output));
        m_elevatorSubsystem.rotateMotors(output);
    }

    // when button not pressed stop rotating
    @Override
    public void end(boolean interrupted) {
        m_elevatorSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return false; // Runs until interrupted
    }
}
