package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import java.util.Optional;

public class AlignRobot extends Command {
    private final DriveSubsystem m_driveSubsystem;
    private final VisionSubsystem m_visionSubsystem;
    private final int targetTagId;

    public AlignRobot(DriveSubsystem m_drivesubsystem, VisionSubsystem m_visionsubsystem, int targetTagId) {
        this.m_driveSubsystem = m_drivesubsystem;
        this.m_visionSubsystem = m_visionsubsystem;
        this.targetTagId = targetTagId;

        addRequirements(m_drivesubsystem, m_visionsubsystem);
    }

    @Override
    public void execute() {
    // Get the offset to the AprilTag
        Optional<Pose2d> offsetPose = m_visionSubsystem.calculateOffsetToTag(targetTagId);

        if (offsetPose.isPresent()) {
            Pose2d offset = offsetPose.get();

            // Get the rotation offset
            double offsetAngle = offset.getRotation().getDegrees();

            // Rotate the robot to correct the angle offset
            if (Math.abs(offsetAngle) > 1.0) { // Tolerance for stopping
                // Create a new SwerveModuleState with the rotation speed for all modules
                // Maybe change to PID later !!!!!!!!!
                SwerveModuleState[] desiredStates = new SwerveModuleState[]{
                    new SwerveModuleState(0, Rotation2d.fromDegrees(offsetAngle * 0.05)), // Only need rotation speed
                    new SwerveModuleState(0, Rotation2d.fromDegrees(offsetAngle * 0.05)),
                    new SwerveModuleState(0, Rotation2d.fromDegrees(offsetAngle * 0.05)),
                    new SwerveModuleState(0, Rotation2d.fromDegrees(offsetAngle * 0.05))
                };
                m_driveSubsystem.setModuleStates(desiredStates);  // Set the swerve module states
            } else {
                // Stop rotating when the angle offset is small enough
                m_driveSubsystem.setModuleStates(new SwerveModuleState[4]); // Stop the robot
            }
        }
    }

    @Override
    public boolean isFinished() {
        // End the command when the robot is aligned (when the angle offset is small enough)
        return Math.abs(m_visionSubsystem.calculateOffsetToTag(targetTagId).get().getRotation().getDegrees()) < 1.0;
    }

    @Override
    public void end(boolean interrupted) {
        SwerveModuleState[] stopStates = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            stopStates[i] = new SwerveModuleState(0.0, new Rotation2d()); // Stop each module (Rotation2d = 0 speed)
        }
        m_driveSubsystem.setModuleStates(stopStates);
    }
}
