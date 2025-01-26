package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeClawSubsystem;

public class RotateClaw extends Command {
    private final IntakeClawSubsystem m_intakeClawSubsystem;
    private final double m_targetPosition;

    public RotateClaw(IntakeClawSubsystem subsystem, double targetPosition) {
        m_intakeClawSubsystem = subsystem;
        m_targetPosition = targetPosition;

        // Declare subsystem dependencies
        addRequirements(m_intakeClawSubsystem); 
    }

    @Override
    public void initialize() {
        // Start moving to the target position
        m_intakeClawSubsystem.setDesiredPosition(m_targetPosition);
    }

    @Override
    public void execute() {
        //m_intakeClawSubsystem.setDesiredPosition(m_targetPosition); // No need to call because of pid controller handling it in initialize
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the motor when the command ends
        m_intakeClawSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        // Check if the position is within tolerance
        return m_intakeClawSubsystem.isAtSetpoint();
    }
}
