package frc.robot.commands;

import java.util.Optional;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class AlignRobot extends Command {
    private final DriveSubsystem m_driveSubsystem;
    private final VisionSubsystem m_visionSubsystem;
    private final int targetTagId;
    
    public AlignRobot(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, int targetTagId) {
        m_driveSubsystem = driveSubsystem;
        m_visionSubsystem = visionSubsystem;
        this.targetTagId = targetTagId;
        
        addRequirements(m_driveSubsystem, m_visionSubsystem); // Subsystem dependencies
    } 

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double offsetAngle = m_visionSubsystem.getAngleOffset(targetTagId);
        m_driveSubsystem.drive(0, 0, -offsetAngle, false); // Only turn, no driving
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        double offsetAngle = m_visionSubsystem.getAngleOffset(targetTagId);
        return Math.abs(offsetAngle) < 1.0;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_driveSubsystem.drive(0, 0, 0, false); // Maybe not necessary
    }

}
