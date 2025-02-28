package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClawWheelsSubsystem;

/** Spins both wheels to shoot algae. */
public class ShootAlgae extends Command {
    private final ClawWheelsSubsystem m_clawWheels;
    private final double m_speed;

    public ShootAlgae(ClawWheelsSubsystem subsystem, double speed) {
        m_clawWheels = subsystem;
        m_speed = speed;

        addRequirements(m_clawWheels);
    }

    @Override
    public void initialize() {
        //m_clawWheels.spinWheelsSync(m_speed);
        m_clawWheels.spinTopWheel(m_speed);
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