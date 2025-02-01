// Needs to be changed to PID
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorMove extends Command {
    private final ElevatorSubsystem m_elevatorSubsystem;
    private final double m_speed;

    public ElevatorMove(ElevatorSubsystem subsystem, double speed) {
        m_elevatorSubsystem = subsystem;
        m_speed = speed;
        addRequirements(subsystem);
    }

    // when button pressed rotate
    @Override
    public void execute() {
        m_elevatorSubsystem.rotateMotors(m_speed);
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
