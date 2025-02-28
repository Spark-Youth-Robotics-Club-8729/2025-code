package frc.robot.commands;

import frc.robot.subsystems.ClawWheelsSubsystem;
import frc.robot.Constants.ClawWheelsConstants;
import edu.wpi.first.wpilibj2.command.Command;

/** A command that runs the top wheel at a low speed continuously */
public class ClawWheelsStall extends Command {
    private final ClawWheelsSubsystem clawWheelsSubsystem;
    private final double m_speed;

    public ClawWheelsStall(ClawWheelsSubsystem subsystem, double speed) {
        this.clawWheelsSubsystem = subsystem;
        m_speed = -speed;
        addRequirements(clawWheelsSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        // Ensure the top wheel keeps spinning at a low speed
        clawWheelsSubsystem.spinTopWheel(m_speed);
    }

    @Override
    public void end(boolean interrupted) {
        clawWheelsSubsystem.stopAll();
    }
}