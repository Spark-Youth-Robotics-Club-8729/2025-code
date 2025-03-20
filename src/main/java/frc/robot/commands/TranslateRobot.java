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

public class TranslateRobot extends Command {
    private final DriveSubsystem m_driveSubsystem;
    private final VisionSubsystem m_visionSubsystem;

    public TranslateRobot(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
        m_driveSubsystem = driveSubsystem;
        m_visionSubsystem = visionSubsystem;
        // Add check coral reef side (ex, boolean right --> true means right, false means left)
        
        addRequirements(m_driveSubsystem, m_visionSubsystem); // Subsystem dependencies
    } 

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        SmartDashboard.putBoolean("TranslateRobot/Running", true);  // Log running status to Shuffleboard
        m_visionSubsystem.resetPID();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        List<Double> output = m_visionSubsystem.align(0.0);
        
        m_driveSubsystem.drive(0, -output.get(1),0,  false);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_visionSubsystem.YisAtSetpoint();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_driveSubsystem.drive(0, 0, 0, true); // Stop the robot's movement
        SmartDashboard.putBoolean("TranslateRobot/Running", false);  
    }
}