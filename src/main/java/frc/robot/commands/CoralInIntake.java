package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClawWheelsSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Spins the bottom wheel to shoot coral. */
public class CoralInIntake extends Command {
    private final ClawWheelsSubsystem m_clawWheels;
    private double itime;
    private final Timer m_timer;

    public CoralInIntake(ClawWheelsSubsystem subsystem) {
        m_clawWheels = subsystem;
        m_timer = new Timer();
        addRequirements(m_clawWheels);
    }

    @Override
    public void initialize() {
        m_timer.reset(); 
        m_timer.start();
        itime = 0.0;
    }


    @Override
    public void execute() {
        if (m_clawWheels.coralInIntake() && itime==0.0) {
            itime = m_timer.get();
        } 
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        if (itime != 0.0 && (m_timer.get()-itime)>0.5) {
            return true;
        }

        return false;
    }
}
