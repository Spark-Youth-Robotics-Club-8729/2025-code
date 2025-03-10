package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ResetEncoders extends Command {
    private final ElevatorSubsystem m_elevatorSubsystem;

    public ResetEncoders(ElevatorSubsystem subsystem) {
        m_elevatorSubsystem = subsystem;
    }

    @Override
    public void initialize() {
        m_elevatorSubsystem.resetEncoders();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
