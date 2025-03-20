package frc.robot.commands;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AlignRobot extends Command {
    private final DriveSubsystem m_driveSubsystem;
    private final VisionSubsystem m_visionSubsystem;
    private final boolean m_right;

    public AlignRobot(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, boolean right) {
        m_driveSubsystem = driveSubsystem;
        m_visionSubsystem = visionSubsystem;
        // Add check coral reef side (ex, boolean right --> true means right, false means left)
        m_right = right; //right = true, then right; right = false, then left
        
        addRequirements(m_driveSubsystem, m_visionSubsystem); // Subsystem dependencies
    } 

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        SmartDashboard.putBoolean("AlignRobot/Running", true);  // Log running status to Shuffleboard
        m_visionSubsystem.resetPID();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        List<Double> output = m_visionSubsystem.alignPID(0.0); //either left or right
        // if (m_right) {
        //     output = m_visionSubsystem.alignPID(VisionConstants.yOffsetRight); //either left or right
        // } else {
        //     output = m_visionSubsystem.alignPID(VisionConstants.yOffsetLeft); //either left or right

        // }
        
        m_driveSubsystem.drive(output.get(1), output.get(2), output.get(0), false);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_visionSubsystem.YisAtSetpoint() && m_visionSubsystem.AngleisAtSetpoint() && m_visionSubsystem.XisAtSetpoint();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_driveSubsystem.drive(0, 0, 0, true); // Stop the robot's movement
        SmartDashboard.putBoolean("AlignRobot/Running", false);  
    }
}