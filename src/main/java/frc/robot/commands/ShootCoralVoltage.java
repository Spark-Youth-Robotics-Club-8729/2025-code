package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClawWheelsSubsystem;

/** Spins the bottom wheel to shoot coral. */
public class ShootCoralVoltage extends Command {
    private final ClawWheelsSubsystem m_clawWheels;
    private final double m_voltage;

    public ShootCoralVoltage(ClawWheelsSubsystem subsystem, double voltage) {
        m_clawWheels = subsystem;
        m_voltage = voltage;
        addRequirements(m_clawWheels);
    }

    @Override
    public void initialize() {
        m_clawWheels.spinBottomWheelVoltage(m_voltage);
    }

    @Override
    public void end(boolean interrupted) {
        m_clawWheels.stopAll();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
