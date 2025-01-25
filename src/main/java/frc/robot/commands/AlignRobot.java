package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AlignRobot extends Command {
    private final DriveSubsystem m_driveSubsystem;
    private final VisionSubsystem m_visionSubsystem;
    private final int targetTagId;
    private final Timer m_timer;  // Timer to track elapsed time
    private static final double TIMEOUT = 5.0; // Timeout after 5 seconds

    public AlignRobot(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, int targetTagId) {
        m_driveSubsystem = driveSubsystem;
        m_visionSubsystem = visionSubsystem;
        this.targetTagId = targetTagId;
        m_timer = new Timer();  // Initialize the timer
        
        addRequirements(m_driveSubsystem, m_visionSubsystem); // Subsystem dependencies
    } 

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_timer.reset();  // Reset the timer
        m_timer.start();  // Start the timer
        SmartDashboard.putBoolean("AlignRobot/Running", true);  // Log running status to Shuffleboard
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double offsetAngle = m_visionSubsystem.getAngleOffset(14);
        //m_driveSubsystem.drive(0, 0, -offsetAngle, false); // Only turn, no driving
        m_driveSubsystem.drive(0, 0, 0.5, false);

        SmartDashboard.putNumber("Offset", offsetAngle);
        SmartDashboard.putNumber("Robot Angle", m_driveSubsystem.getHeading());
        SmartDashboard.putNumber("Robot Turn Rate", m_driveSubsystem.getTurnRate());
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        double offsetAngle = m_visionSubsystem.getAngleOffset(targetTagId);
        
        // Stop the command if the robot is aligned (offset angle is small enough) or if the timeout is reached
        if (Math.abs(offsetAngle) < 1.0) {
            return true;  // Goal reached
        } else if (m_timer.get() > TIMEOUT) {
            return true;  // Timeout reached
        }
        
        return false;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_driveSubsystem.drive(0, 0, 0, false); // Stop the robot's movement
        SmartDashboard.putBoolean("AlignRobot/Running", false);  
    }
}