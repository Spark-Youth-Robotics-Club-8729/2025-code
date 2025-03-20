package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClawWheelsConstants;
import frc.robot.subsystems.ClawWheelsSubsystem;

/** Spins the bottom wheel to shoot coral. */
public class ShootCoral extends Command {
    private final ClawWheelsSubsystem m_clawWheels;
    private final double m_speed;

    public ShootCoral(ClawWheelsSubsystem subsystem, double speed) {
        m_clawWheels = subsystem;
        m_speed = speed;
        addRequirements(m_clawWheels);
    }

    @Override
    public void initialize() {
        m_clawWheels.spinBottomWheel(m_speed);
        m_clawWheels.spinTopWheel(ClawWheelsConstants.kIntakeAlgaeSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        m_clawWheels.stopAll();
        m_clawWheels.spinTopWheel(ClawWheelsConstants.kIntakeAlgaeStall);

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
