package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimberSet extends Command {
    private final ClimbSubsystem m_climbersubsystem;
    private final double m_angle;

    /** Creates a new ClimberSet. */
    public ClimberSet(ClimbSubsystem climbersubsystem, double angle) {
        m_angle = angle;
        m_climbersubsystem = climbersubsystem;
        addRequirements(m_climbersubsystem);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_climbersubsystem.setDesiredAngle(m_angle);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_climbersubsystem.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_climbersubsystem.isAtSetpoint();
    }
}