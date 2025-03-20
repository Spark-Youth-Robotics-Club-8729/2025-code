package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import java.util.List;


public class AlignRobotPID extends Command{
    private final DriveSubsystem m_DriveSubsystem;
    private final VisionSubsystem m_VisionSubsystem;
    

    public AlignRobotPID(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
        m_DriveSubsystem = driveSubsystem;
        m_VisionSubsystem = visionSubsystem;


        addRequirements(m_DriveSubsystem);
    }

    @Override
    public void initialize() {
        m_VisionSubsystem.resetPID();           
    }

    @Override
    public void execute() {
        List<Double> result = m_VisionSubsystem.applyPIDMarch19Code(0);
        m_DriveSubsystem.drive(result.get(0), result.get(1), result.get(2), false);
    }

    @Override
    public void end(boolean interrupted) {
        m_DriveSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return m_VisionSubsystem.isAtSetpoint();
    }
}
