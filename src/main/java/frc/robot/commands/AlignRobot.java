package frc.robot.commands;

import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AlignRobot extends Command {
    private final DriveSubsystem m_driveSubsystem;
    private final VisionSubsystem m_visionSubsystem;

    public AlignRobot(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
        m_driveSubsystem = driveSubsystem;
        m_visionSubsystem = visionSubsystem;
        
        addRequirements(m_driveSubsystem, m_visionSubsystem); // Subsystem dependencies
    } 

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        SmartDashboard.putBoolean("AlignRobot/Running", true);  // Log running status to Shuffleboard
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double offsetAngle = m_visionSubsystem.getAngleOffset(VisionConstants.kAprilTagIds);

        double turnRot = (-offsetAngle) * 0.05;
        m_driveSubsystem.drive(0, 0, turnRot, true);

        SmartDashboard.putNumber("Offset", offsetAngle);
        SmartDashboard.putNumber("Robot Angle", m_driveSubsystem.getHeading());
        SmartDashboard.putNumber("Robot Turn Rate", m_driveSubsystem.getTurnRate());
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        double offsetAngle = m_visionSubsystem.getAngleOffset(VisionConstants.kAprilTagIds);
        
        // Stop the command if the robot is aligned (offset angle is small enough) or if the timeout is reached
        if (Math.abs(offsetAngle) < 0.1) {
            return true;  // Goal reached
        }
        
        return false;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_driveSubsystem.drive(0, 0, 0, true); // Stop the robot's movement
        SmartDashboard.putBoolean("AlignRobot/Running", false);  
    }
}