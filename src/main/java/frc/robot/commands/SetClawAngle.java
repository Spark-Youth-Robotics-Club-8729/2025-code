package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeClawSubsystem;

public class SetClawAngle extends Command {
    private final IntakeClawSubsystem m_intakeClawSubsystem;
    private final double m_targetPosition;

    public SetClawAngle(IntakeClawSubsystem subsystem, double targetPosition) {
        m_intakeClawSubsystem = subsystem;
        m_targetPosition = targetPosition;

        // Declare subsystem dependencies
        // addRequirements(m_intakeClawSubsystem); //IntakeSubsystem is not a subsystem
    }

    @Override
    public void initialize() {
        // Start moving to the target position
        m_intakeClawSubsystem.setDesiredPosition(m_targetPosition);
    }

    @Override
    public void execute() {
        // Continuously call the setPosition method in case of updates
        m_intakeClawSubsystem.setDesiredPosition(m_targetPosition);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the motor when the command ends
        m_intakeClawSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        // Check if the position is within tolerance
        return Math.abs(m_intakeClawSubsystem.getPosition() - m_targetPosition) <= 0.1;
    }
}
