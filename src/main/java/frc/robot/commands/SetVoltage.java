package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class SetVoltage extends Command {
    private final ElevatorSubsystem m_elevatorSubsystem;

    public SetVoltage(ElevatorSubsystem subsystem) {
        m_elevatorSubsystem = subsystem;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_elevatorSubsystem.setVoltage(0.3765);
    }


    @Override
    public void end(boolean interrupted) {
        m_elevatorSubsystem.setVoltage(0.0);
    }

}
