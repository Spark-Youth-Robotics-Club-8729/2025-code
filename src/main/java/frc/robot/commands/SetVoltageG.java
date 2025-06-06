package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class SetVoltageG extends Command {
    private final ElevatorSubsystem m_elevatorSubsystem;
    private final double m_voltage;

    public SetVoltageG(ElevatorSubsystem subsystem, double voltage) {
        m_elevatorSubsystem = subsystem;
        m_voltage = voltage;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_elevatorSubsystem.setVoltage(m_voltage);
    }


    @Override
    public void end(boolean interrupted) {
        m_elevatorSubsystem.setVoltage(0.0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
